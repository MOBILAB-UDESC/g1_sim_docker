import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
import os
from collections import deque

# --- Deploy-order joint layout (matches your confirmed ros2_control topics) ---
DEPLOY_JOINTS = [
    "left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint",
    "left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
    "right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint",
    "right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
    "waist_yaw_joint","waist_roll_joint","waist_pitch_joint",
    "left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint",
    "left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint",
    "right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint",
    "right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint",
]

# policy index k -> deploy index, straight from unitree_rl_lab's deploy.yaml
JOINT_IDS_MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23,
                 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]

# Per-policy-index arrays, straight from deploy.yaml
STIFFNESS = [100,100,100,150,40,40,100,100,100,150,40,40,200,200,200,
             40,40,40,40,40,40,40,40,40,40,40,40,40,40]
DAMPING = [2,2,2,4,2,2,2,2,2,4,2,2,5,5,5,10,10,10,10,10,10,10,10,10,10,10,10,10,10]
DEFAULT_POS = [-0.1,-0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.3,0.3,0.3,0.3,-0.2,-0.2,
               0.25,-0.25,0.0,0.0,0.0,0.0,0.97,0.97,0.15,-0.15,0.0,0.0,0.0,0.0]
ACTION_SCALE = 0.25
STEP_DT = 0.02
HISTORY_LEN = 5
N = 29

CONTROLLER_GROUPS = {
    "left_leg_position_controller":  DEPLOY_JOINTS[0:6],
    "right_leg_position_controller": DEPLOY_JOINTS[6:12],
    "waist_position_controller":     DEPLOY_JOINTS[12:15],
    "left_arm_position_controller":  DEPLOY_JOINTS[15:22],
    "right_arm_position_controller": DEPLOY_JOINTS[22:29],
}


def quat_to_gravity(qx, qy, qz, qw):
    """Projected gravity vector in body frame from orientation quaternion."""
    gx = 2*(qx*qz - qw*qy)
    gy = 2*(qy*qz + qw*qx)
    gz = 1 - 2*(qx*qx + qy*qy)
    return np.array([-gx, -gy, -gz], dtype=np.float32)


class PolicyRunner(Node):
    def __init__(self):
        super().__init__('g1_policy_runner')

        self._debug_count = 0

        onnx_path = os.path.expanduser('/workspace/src/g1_policy_runner/g1_policy_runner/policy/policy.onnx') 
        self.session = ort.InferenceSession(onnx_path)
        self.input_name = self.session.get_inputs()[0].name
        self.get_logger().info(f"Loaded policy: {onnx_path}")

        # deploy-order joint state, filled from /joint_states
        self.joint_pos = dict.fromkeys(DEPLOY_JOINTS, 0.0)
        self.joint_vel = dict.fromkeys(DEPLOY_JOINTS, 0.0)
        self.have_joint_state = False

        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.gravity = np.array([0, 0, -1], dtype=np.float32)
        self.have_imu = False

        self.cmd_vel = np.zeros(3, dtype=np.float32)  # vx, vy, yaw_rate

        # history buffers, oldest -> newest
        self.hist_ang_vel = deque([np.zeros(3, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_gravity = deque([np.array([0, 0, -1], dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_cmd = deque([np.zeros(3, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_joint_pos = deque([np.zeros(N, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_joint_vel = deque([np.zeros(N, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN)
        self.hist_action = deque([np.zeros(N, dtype=np.float32)] * HISTORY_LEN, maxlen=HISTORY_LEN)

        self.last_action = np.zeros(N, dtype=np.float32)

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Imu, '/pelvis_imu_broadcaster/imu', self.imu_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.pubs = {
            name: self.create_publisher(Float64MultiArray, f'/{name}/commands', 10)
            for name in CONTROLLER_GROUPS
        }
        self.ready_pub = self.create_publisher(Bool, '/policy_runner/ready', 10)

        self.enabled = False  # gate: only actuate once we've seen real joint_states + imu
        self.timer = self.create_timer(STEP_DT, self.step)

    def joint_state_cb(self, msg: JointState):
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.joint_pos:
                self.joint_pos[name] = pos
                self.joint_vel[name] = vel
        self.have_joint_state = True

    def imu_cb(self, msg: Imu):
        self.ang_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float32)
        q = msg.orientation
        self.gravity = quat_to_gravity(q.x, q.y, q.z, q.w)
        self.have_imu = True

    def cmd_vel_cb(self, msg: Twist):
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)

    def build_obs(self):
        deploy_pos = np.array([self.joint_pos[n] for n in DEPLOY_JOINTS], dtype=np.float32)
        deploy_vel = np.array([self.joint_vel[n] for n in DEPLOY_JOINTS], dtype=np.float32)
        policy_pos = deploy_pos[JOINT_IDS_MAP]
        policy_vel = deploy_vel[JOINT_IDS_MAP]

        joint_pos_rel = (policy_pos - np.array(DEFAULT_POS, dtype=np.float32))
        joint_vel_rel = policy_vel * 0.05

        self.hist_ang_vel.append(self.ang_vel * 0.2)
        self.hist_gravity.append(self.gravity)
        self.hist_cmd.append(self.cmd_vel)
        self.hist_joint_pos.append(joint_pos_rel)
        self.hist_joint_vel.append(joint_vel_rel)
        self.hist_action.append(self.last_action.copy())

        obs = np.concatenate([
            np.concatenate(list(self.hist_ang_vel)),
            np.concatenate(list(self.hist_gravity)),
            np.concatenate(list(self.hist_cmd)),
            np.concatenate(list(self.hist_joint_pos)),
            np.concatenate(list(self.hist_joint_vel)),
            np.concatenate(list(self.hist_action)),
        ]).astype(np.float32)
        return obs

    def step(self):
        if not (self.have_joint_state and self.have_imu):
            return  # wait for real sensor data before actuating

        obs = self.build_obs()
        action = self.session.run(None, {self.input_name: obs[None, :]})[0][0]
        action = np.clip(action, -5.0, 5.0)
        if self._debug_count < 20:
            self._debug_count += 1
            self.get_logger().info(
                f"step {self._debug_count}: "
                f"action min/max/mean={action.min():.3f}/{action.max():.3f}/{action.mean():.3f} "
                f"gravity={self.gravity.round(3)} ang_vel={self.ang_vel.round(3)}"
            )
        self.last_action = action.copy()

        target_policy = np.array(DEFAULT_POS, dtype=np.float32) + action * ACTION_SCALE

        target_deploy = np.zeros(N, dtype=np.float32)
        for k in range(N):
            target_deploy[JOINT_IDS_MAP[k]] = target_policy[k]
        target_deploy = np.clip(target_deploy, -2.8, 2.8)

        if self._debug_count <= 20:
            left_leg_idx = [0, 1, 2, 3, 4, 5]
            current = np.array([self.joint_pos[DEPLOY_JOINTS[i]] for i in left_leg_idx])
            target = target_deploy[left_leg_idx]
            self.get_logger().info(
                f"  left leg current={current.round(3)} target={target.round(3)} "
                f"delta={(target-current).round(3)}"
            )

        for name, joints in CONTROLLER_GROUPS.items():
            values = [float(target_deploy[DEPLOY_JOINTS.index(j)]) for j in joints]
            msg = Float64MultiArray()
            msg.data = values
            self.pubs[name].publish(msg)

        if not self.enabled:
            self.enabled = True
            self.get_logger().info("Policy loop active — publishing commands.")
        self.ready_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = PolicyRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()