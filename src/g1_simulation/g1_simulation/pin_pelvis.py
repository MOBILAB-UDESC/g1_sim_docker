#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mujoco_ros2_control.srv import GetBodyState, SetBodyPose
from std_srvs.srv import Trigger


class PelvisAnchor(Node):
    def __init__(self):
        super().__init__('pelvis_anchor')
        self.get_cli = self.create_client(GetBodyState, '/mujoco_get_body_state')
        self.set_cli = self.create_client(SetBodyPose, '/mujoco_set_body_pose')
        self.reset_cli = self.create_client(Trigger, '/mujoco_reset')
        self.get_logger().info("Waiting for mujoco services...")
        self.get_cli.wait_for_service()
        self.set_cli.wait_for_service()
        self.reset_cli.wait_for_service()

        self.declare_parameter('ready_topic', '/policy_runner/ready')
        ready_topic = self.get_parameter('ready_topic').value

        # Force a reset to the known-good keyframe pose before capturing
        # anything - otherwise a race with physics already stepping can
        # anchor to an already-sagging pose instead of the correct stance.
        self.get_logger().info("Resetting sim to keyframe pose before anchoring...")
        reset_future = self.reset_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, reset_future)

        self.pinned = True
        self.pose = None
        self.capture_initial_pose()

        self.get_logger().info(f"Waiting for ready signal on {ready_topic}")
        self.create_subscription(Bool, ready_topic, self.ready_cb, 10)

        self.pin_req = SetBodyPose.Request()
        self.pin_req.body_name = 'pelvis'
        self.pin_req.x = self.pose.position.x
        self.pin_req.y = self.pose.position.y
        self.pin_req.z = self.pose.position.z
        self.pin_req.qw = self.pose.orientation.w
        self.pin_req.qx = self.pose.orientation.x
        self.pin_req.qy = self.pose.orientation.y
        self.pin_req.qz = self.pose.orientation.z

        # Timer-based pinning instead of a background thread - everything
        # runs through the single rclpy.spin(node) call in main(), so there
        # is no second thread competing to spin the same node's executor.
        self.pin_timer = self.create_timer(0.01, self.pin_tick)

    def capture_initial_pose(self):
        req = GetBodyState.Request()
        req.body_name = 'pelvis'
        future = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.pose = future.result().pose
        self.get_logger().info(f"Anchoring pelvis at z={self.pose.position.z:.3f}")

    def ready_cb(self, msg: Bool):
        if msg.data and self.pinned:
            self.pinned = False
            self.pin_timer.cancel()
            self.get_logger().info("Ready signal received — releasing pelvis.")

    def pin_tick(self):
        if not self.pinned:
            return
        # Fire-and-forget: no blocking spin call here, since we are already
        # inside the node's normal timer callback (itself driven by the one
        # spin() in main()).
        self.set_cli.call_async(self.pin_req)


def main():
    rclpy.init()
    node = PelvisAnchor()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
