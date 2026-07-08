#!/usr/bin/env python3

# test node use for gaze node

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class GazeTfBroadcaster(Node):
    def __init__(self):
        super().__init__('gaze_tf_broadcaster')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'gaze_point')
        self.declare_parameter('x', 2.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.0)
        self.declare_parameter('rate_hz', 20.0)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / rate_hz, self.broadcast)

        self.get_logger().info(
            f'GazeTfBroadcaster.-> publishing {self.parent_frame} -> '
            f'{self.child_frame} at {rate_hz:.0f} Hz'
        )

    def broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = float(self.get_parameter('x').value)
        t.transform.translation.y = float(self.get_parameter('y').value)
        t.transform.translation.z = float(self.get_parameter('z').value)
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GazeTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
