import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist
import os
from collections import deque

# --- Deploy-order joint layout, same convention as g1_policy_runner ---
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
N_ALL = 29          # all joints, used for pos/vel observation terms
N_ACTIONS = 15       # legs + waist only - this policy does not own the arms

# Straight SDK/deploy order for the 15 owned joints - no joint_ids_map needed
# for this policy (unlike unitree_rl_lab's policy, confirmed from source).
DEFAULTS = np.array([-0.1,0.0,0.0,0.3,-0.2,0.0, -0.1,0.0,0.0,0.3,-0.2,0.0, 0.0,0.0,0.0], dtype=np.float32)
KPS = np.array([150,150,150,200,40,40, 150,150,150,200,40,40, 250,250,250], dtype=np.float32)
KDS = np.array([2,2,2,4,2,2, 2,2,2,4,2,2, 5,5,5], dtype=np.float32)

ACTION_SCALE = 0.25
BALANCE_CMD_NORM = 0.05
ANG_VEL_SCALE = 0.5
DOF_VEL_SCALE = 0.05
CMD_SCALE = np.array([2.0, 2.0, 0.5], dtype=np.float32)  # vx, vy, yaw

# Variant: gr00t_wbc_h066_p000
CMD_HEIGHT = 0.76
CMD_BODY_PITCH = 0.0

SINGLE_OBS = 86
HISTORY = 6
STEP_DT = 0.02

CONTROLLER_GROUPS = {
    "left_leg_position_controller":  DEPLOY_JOINTS[0:6],
    "right_leg_position_controller": DEPLOY_JOINTS[6:12],
    "waist_position_controller":     DEPLOY_JOINTS[12:15],
}


def quat_to_gravity(qx, qy, qz, qw):
    gx = 2*(qx*qz - qw*qy)
    gy = 2*(qy*qz + qw*qx)
    gz = 1 - 2*(qx*qx + qy*qy)
    return np.array([-gx, -gy, -gz], dtype=np.float32)


class Gr00tWbcRunner(Node):
    def __init__(self):
        super().__init__('gr00t_wbc_runner')

        self._debug_count = 0
        models_dir = os.path.expanduser('/workspace/src/gr00t_wbc_runner/gr00t_wbc_runner/models')
        self.walk_session = ort.InferenceSession(os.path.join(models_dir, 'model_walk.onnx'))
        self.balance_session = ort.InferenceSession(os.path.join(models_dir, 'model_balance.onnx'))
        self.walk_input = self.walk_session.get_inputs()[0].name
        self.balance_input = self.balance_session.get_inputs()[0].name
        self.get_logger().info("Loaded gr00t_wbc walk + balance models (variant h066_p000)")

        self.joint_pos = dict.fromkeys(DEPLOY_JOINTS, 0.0)
        self.joint_vel = dict.fromkeys(DEPLOY_JOINTS, 0.0)
        self.have_joint_state = False

        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.gravity = np.array([0, 0, -1], dtype=np.float32)
        self.have_imu = False

        self.cmd_vel = np.zeros(3, dtype=np.float32)  # vx, vy, yaw_rate
        self.last_action = np.zeros(N_ACTIONS, dtype=np.float32)

        # Rolling history of single-timestep 86-dim observations, oldest -> newest
        self.history = deque([np.zeros(SINGLE_OBS, dtype=np.float32)] * HISTORY, maxlen=HISTORY)

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Imu, '/pelvis_imu_broadcaster/imu', self.imu_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        self.pubs = {
            name: self.create_publisher(Float64MultiArray, f'/{name}/commands', 10)
            for name in CONTROLLER_GROUPS
        }
        self.ready_pub = self.create_publisher(Bool, '/gr00t_wbc_runner/ready', 10)

        self.enabled = False
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

    def build_single_obs(self):
        deploy_pos = np.array([self.joint_pos[n] for n in DEPLOY_JOINTS], dtype=np.float32)
        deploy_vel = np.array([self.joint_vel[n] for n in DEPLOY_JOINTS], dtype=np.float32)
        default_full = np.zeros(N_ALL, dtype=np.float32)
        default_full[:N_ACTIONS] = DEFAULTS

        f = np.zeros(SINGLE_OBS, dtype=np.float32)
        f[0:3] = self.cmd_vel * CMD_SCALE
        f[3] = CMD_HEIGHT
        f[4] = 0.0
        f[5] = CMD_BODY_PITCH
        f[6] = 0.0
        f[7:10] = self.ang_vel * ANG_VEL_SCALE
        f[10:13] = self.gravity
        f[13:13+N_ALL] = deploy_pos - default_full
        f[13+N_ALL:13+2*N_ALL] = deploy_vel * DOF_VEL_SCALE
        f[13+2*N_ALL:13+2*N_ALL+N_ACTIONS] = self.last_action
        return f

    def step(self):
        if not (self.have_joint_state and self.have_imu):
            return

        self.history.append(self.build_single_obs())
        obs = np.concatenate(list(self.history)).astype(np.float32)[None, :]

        act_walk = self.walk_session.run(None, {self.walk_input: obs})[0][0]
        act_balance = self.balance_session.run(None, {self.balance_input: obs})[0][0]

        norm = float(np.linalg.norm(self.cmd_vel))
        action = act_walk if norm > BALANCE_CMD_NORM else act_balance

        # Safety clamp - same lesson learned from the last policy's runaway bug.
        action = np.clip(action, -5.0, 5.0)
        self.last_action = action.copy()

        if self._debug_count < 20:
            self._debug_count += 1
            self.get_logger().info(
                f"step {self._debug_count}: mode={'walk' if norm > BALANCE_CMD_NORM else 'balance'} "
                f"action min/max/mean={action.min():.3f}/{action.max():.3f}/{action.mean():.3f} "
                f"gravity={self.gravity.round(3)}"
            )

        target = DEFAULTS + action * ACTION_SCALE
        target = np.clip(target, -2.8, 2.8)  # physical sanity clamp

        for name, joints in CONTROLLER_GROUPS.items():
            idx = [DEPLOY_JOINTS.index(j) for j in joints]
            values = [float(target[DEPLOY_JOINTS.index(j)]) for j in joints]
            msg = Float64MultiArray()
            msg.data = values
            self.pubs[name].publish(msg)

        if not self.enabled:
            self.enabled = True
            self.get_logger().info("gr00t_wbc loop active — publishing commands.")
        self.ready_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = Gr00tWbcRunner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()