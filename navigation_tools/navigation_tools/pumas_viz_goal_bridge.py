#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from pumas_interfaces.action import PumasNav
from pumas_interfaces.msg import StartAndEndJoints
from rclpy.action import ActionClient
from rclpy.node import Node


class RvizGoalBridge(Node):
    def __init__(self):
        super().__init__("rviz_goal_bridge")
        self.cli = ActionClient(self, PumasNav, "/pumasnav")

        self.sub = self.create_subscription(
            PoseStamped,
            "/move_base_simple/goal",
            self.cb_goal,
            10,
        )

        self.get_logger().info("RvizGoalBridge started")

    def cb_goal(self, msg: PoseStamped):
        self.get_logger().info(
            f"Received RViz goal: x={msg.pose.position.x}, y={msg.pose.position.y}"
        )

        if not self.cli.wait_for_server():
            self.get_logger().error("PumasNav action server not available")
            return

        goal = PumasNav.Goal()
        goal.goal = msg
        goal.arm_joints = StartAndEndJoints()
        goal.use_arm = False
        # goal.patience = True
        # goal.proximity_criterion = 2.0

        future = self.cli.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("PumasNav goal rejected")
            return

        self.get_logger().info("PumasNav goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"feedback: state={fb.state_name}, "
            f"dist={fb.remaining_distance:.3f}, "
            f"near={fb.near_goal_reached}, "
            f"msg={fb.message}"
        )

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"result: success={result.success}, "
            f"near_goal_reached={result.near_goal_reached}, "
            f"message={result.message}"
        )


def main():
    rclpy.init()
    node = RvizGoalBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
