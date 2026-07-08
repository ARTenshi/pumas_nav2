#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
from typing import Optional, Union

import numpy as np
import rclpy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Pose2D, PoseArray, PoseStamped
from pumas_interfaces.action import GazeHead, PumasNav
from pumas_interfaces.msg import Joints, StartAndEndJoints
from pumas_interfaces.srv import ParamReadWrite
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, Float32MultiArray
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

default_arm_pose = {
    'arm_flex_joint': -0.26,  # default is 0.0
    'arm_lift_joint': 0.0,
    'arm_roll_joint': -1.57,
    'wrist_flex_joint': -1.57,
    'wrist_roll_joint': 0.0,
    'head_pan_joint': 0.0,
    'head_tilt_joint': np.deg2rad(0.0),
}


class NavModule:
    """Navigation Module for the robot"""

    def __init__(self, node: Optional[Union[str, Node]] = None):

        context = rclpy.get_default_context()
        if not context.ok():
            rclpy.init()

        if node is None:
            self._node: Node = rclpy.create_node('nav_module_node')
            self._external_node = False
        elif isinstance(node, str):
            self._node: Node = rclpy.create_node(node)
            self._external_node = False
        elif isinstance(node, Node):
            self._node: Node = node
            self._external_node = True
        else:
            raise TypeError(
                "NavModule.__init__: 'node' must be of type str or rclpy.node.Node")

        self.marker = Marker()
        self.marker_num = 0

        self.robot_stop = False

        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None
        self.motion_execution_time = 0.0  # for motion_synth arm/head reached time
        self.via_points = None  # list of Pose2D the path should pass through

        # Publishers
        self.pub_marker = self.create_publisher(Marker, '/nav_goal_marker', 10)

        # Relative holonomic move (go_rel): base-frame [x, y, yaw].
        self.pub_goal_rel_pose = self.create_publisher(
            Float32MultiArray, '/simple_move/goal_rel_pose', 10
        )
        self.pub_robot_stop = self.create_publisher(
            Empty, '/navigation/stop', 10)

        # Recovery-pose publishers: when motion_synth is NOT used, the body is
        self.pub_arm_goal = self.create_publisher(
            Float32MultiArray, '/hardware/arm/goal_pose', 10)
        self.pub_torso_goal = self.create_publisher(
            Float32, '/hardware/torso/goal_pose', 10)
        self.pub_head_goal_pose = self.create_publisher(
            Float32MultiArray, '/hardware/head/goal_pose', 10
        )

        self.create_subscription(Empty, '/stop', self.callback_stop, 10)

        # Service Clients
        self.param_rw_client = self.create_client(
            ParamReadWrite, '/param_read_write')

        # pumasnav action cli
        self.nav_action_client = ActionClient(
            self._node, PumasNav, '/pumasnav')
        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None

        self._action_feedback = None
        self._action_done = False
        self._action_success = False
        self._action_near_goal = False
        self._action_message = ''
        self._near_goal_callback = None

        # Gaze action client
        self.gaze_action_client = ActionClient(
            self._node, GazeHead, '/gaze_head')
        self._gaze_goal_handle = None
        self._gaze_active = False

        # Relative move (go_rel): command goes out on pub_goal_rel_pose, and
        self.create_subscription(
            GoalStatus, '/simple_move/goal_reached', self._move_goal_reached_callback, 10
        )
        self._move_done = False
        self._move_success = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self._node)

        self._load_default_arm_pose()  # call once here -> arm moves default pose

        self.get_logger().info('NavModule.->initialized')

    def __getattr__(self, name):
        return getattr(self._node, name)

    def call_param_rw(
        self,
        node_name,
        param_name,
        param_value: str = '',
        write: bool = False,
        timeout_sec: float = 3.0,
    ):
        req = ParamReadWrite.Request()
        req.node_name = node_name
        req.param_name = param_name
        req.write = write
        req.value = param_value

        future = self.param_rw_client.call_async(req)

        if self._external_node:
            done = threading.Event()
            future.add_done_callback(lambda _f: done.set())
            if not done.wait(timeout=timeout_sec):
                self.get_logger().error('NavModule.->param_read_write call timed out')
                return None
        else:
            executor = SingleThreadedExecutor()
            executor.add_node(self._node)
            try:
                executor.spin_until_future_complete(
                    future, timeout_sec=timeout_sec)
            finally:
                executor.remove_node(self._node)
                executor.shutdown()
            if not future.done():
                self.get_logger().error('NavModule.->param_read_write call timed out')
                return None

        if future.result() is not None:
            return future.result().param_value
        else:
            self.get_logger().error('NavModule.->param_read_write service call failed')
            return None

    def reset_action_state(self):
        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None
        self._action_feedback = None
        self._action_done = False
        self._action_success = False
        self._action_near_goal = False
        self._action_message = ''

    def nav_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self._action_feedback = feedback

        if feedback.near_goal_reached and not self._action_near_goal:
            self._action_near_goal = True
            if self._near_goal_callback is not None:
                try:
                    self._near_goal_callback(feedback)
                except Exception as e:
                    self.get_logger().error(
                        f'NavModule.->near_goal_callback failed: {e}')
        self.get_logger().info(
            f'Nav feedback: state={feedback.state_name}, '
            f'dist={feedback.remaining_distance:.3f}, '
            f'near={feedback.near_goal_reached}, '
            f'msg={feedback.message}'
        )

    def nav_result_callback(self, future):
        result = future.result().result
        self._action_done = True
        self._action_success = bool(result.success)
        self._action_near_goal = bool(result.near_goal_reached)
        self._action_message = result.message

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('NavModule.->PumasNav goal rejected')
            self._action_done = True
            self._action_success = False
            self._action_message = 'Goal rejected'
            return

        self.get_logger().info('NavModule.->PumasNav goal accepted')
        self._goal_handle = goal_handle
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.nav_result_callback)

    def cancel_nav_action(self):
        if self._goal_handle is not None:
            self.get_logger().warn('NavModule.->Canceling PumasNav goal')
            self._goal_handle.cancel_goal_async()

    def _move_goal_reached_callback(self, msg):
        if msg.goal_id.id != '-1':  # for go_rel
            return
        self._move_done = True
        self._move_success = msg.status == GoalStatus.SUCCEEDED

    def _gaze_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('NavModule.->GazeHead goal rejected')
            self._gaze_active = False
            return
        self._gaze_goal_handle = goal_handle
        self._gaze_active = True
        self.get_logger().info('NavModule.->GazeHead goal accepted, gaze active')

    def start_gaze(self, gaze_tf_name: str = '', restore_head=None) -> bool:
        if not self.gaze_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('NavModule.->GazeHead action server not available')
            return False

        goal_msg = GazeHead.Goal()
        goal_msg.gaze_tf_name = gaze_tf_name
        if restore_head is not None:
            goal_msg.restore_head = True
            goal_msg.restore_pan = float(restore_head[0])
            goal_msg.restore_tilt = float(restore_head[1])

        self._gaze_goal_handle = None
        self._gaze_active = False

        send_future = self.gaze_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._gaze_goal_response_callback)
        self.get_logger().info(
            f"NavModule.->GazeHead goal sent (tf='{gaze_tf_name or 'default'}')")
        return True

    def cancel_gaze(self):
        if self._gaze_goal_handle is not None:
            self.get_logger().info('NavModule.->Canceling GazeHead goal')
            self._gaze_goal_handle.cancel_goal_async()
        self._gaze_goal_handle = None
        self._gaze_active = False

    def _set_move_head(self, enabled: bool, sync: bool = True):
        value = 'true' if enabled else 'false'
        for node_name in ('head_controller', 'simple_move'):
            if sync:
                self.call_param_rw(
                    node_name=node_name,
                    param_name='move_head',
                    param_value=value,
                    write=True,
                )
            else:
                req = ParamReadWrite.Request()
                req.node_name = node_name
                req.param_name = 'move_head'
                req.write = True
                req.value = value
                self.param_rw_client.call_async(req)
        self.get_logger().info(
            f'NavModule.->move_head set to {value} (head_controller, simple_move)'
        )

    def _set_use_point_cloud(self, enabled: bool):
        value = 'true' if enabled else 'false'

        self.call_param_rw(
            node_name='potential_fields',
            param_name='use_point_cloud',
            param_value=value,
            write=True,
        )
        self.call_param_rw(
            node_name='map_augmenter',
            param_name='use_point_cloud',
            param_value=value,
            write=True,
        )

        self.get_logger().info(f'NavModule.->use_point_cloud set to {value}')

    def callback_stop(self, msg):
        self.robot_stop = True

    def get_global_pose_from_tf(self, target_frame='map', source_frame='base_footprint'):
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw = euler[2]

            # self.get_logger().info(
            #    f"NavModule.->TF Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            # )
            return x, y, yaw

        except TransformException as ex:
            self.get_logger().warn(
                f'NavModule.->TF lookup failed: {target_frame} -> {source_frame}: {ex}'
            )
            return None

    def pose_stamped2pose_2d(self, pose_stamped):
        pose2d = Pose2D()
        pose2d.x = pose_stamped.pose.position.x
        pose2d.y = pose_stamped.pose.position.y
        orientation = pose_stamped.pose.orientation
        euler = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        pose2d.theta = euler[2]
        return pose2d

    def _load_default_arm_pose(self, arm_node='/arm_controller', timeout_sec=3.0):
        """Load the default/recovery arm pose from the arm_controller node's
        params (set via <param> in its launch <node> block):

            torso_default_pose : double      -> arm_lift_joint
            arm_default_pose   : double[4]   -> arm_flex/arm_roll/wrist_flex/wrist_roll

        Head is reset to 0 (as in ROS1). If the params cannot be read (node not
        up / service timeout), fall back to the module-level default_arm_pose.
        Stored in self.default_arm_pose and used for the recovery pose and as
        the base for motion_synth start/end poses.
        """
        pose = dict(default_arm_pose)  # fallback

        cli = self._node.create_client(
            GetParameters, f'{arm_node}/get_parameters')
        if cli.wait_for_service(timeout_sec=timeout_sec):
            req = GetParameters.Request()
            req.names = ['torso_default_pose', 'arm_default_pose']
            future = cli.call_async(req)

            if self._external_node:
                done = threading.Event()
                future.add_done_callback(lambda _f: done.set())
                ok = done.wait(timeout=timeout_sec)
            else:
                rclpy.spin_until_future_complete(
                    self._node, future, timeout_sec=timeout_sec)
                ok = future.done()

            res = future.result() if ok else None
            if res is not None and len(res.values) >= 2:
                torso = res.values[0].double_value
                arm = list(res.values[1].double_array_value)
                if len(arm) >= 4:
                    pose['arm_lift_joint'] = float(torso)
                    pose['arm_flex_joint'] = float(arm[0])
                    pose['arm_roll_joint'] = float(arm[1])
                    pose['wrist_flex_joint'] = float(arm[2])
                    pose['wrist_roll_joint'] = float(arm[3])
                    pose['head_pan_joint'] = 0.0
                    pose['head_tilt_joint'] = 0.0
                    self.get_logger().info(
                        f'NavModule.->default_arm_pose from {arm_node}: {pose}')
                else:
                    self.get_logger().warn(
                        'NavModule.->arm_default_pose has <4 values; using module default'
                    )
            else:
                self.get_logger().warn(
                    f'NavModule.->failed to read {arm_node} params; using module default'
                )
        else:
            self.get_logger().warn(
                f'NavModule.->{arm_node}/get_parameters not available; using module default'
            )

        self.default_arm_pose = pose

    def create_arm_joint_goal(self, joint_poses):
        joint_poses = {**self.default_arm_pose, **(joint_poses or {})}
        joints = Joints()
        joints.arm_lift_joint = joint_poses['arm_lift_joint']
        joints.arm_flex_joint = joint_poses['arm_flex_joint']
        joints.arm_roll_joint = joint_poses['arm_roll_joint']
        joints.wrist_flex_joint = joint_poses['wrist_flex_joint']
        joints.wrist_roll_joint = joint_poses['wrist_roll_joint']
        joints.head_pan_joint = joint_poses['head_pan_joint']
        joints.head_tilt_joint = joint_poses['head_tilt_joint']
        return joints

    def _send_default_arm_pose(self, include_head: bool = True):
        """Move the body to default_arm_pose (used when motion_synth is off).

        Uses self.default_arm_pose (loaded from ROS params at launch, falling
        back to the module-level default_arm_pose).
        """
        pose = self.default_arm_pose
        lift = Float32()
        lift.data = float(pose['arm_lift_joint'])
        self.pub_torso_goal.publish(lift)

        arm = Float32MultiArray()
        arm.data = [
            float(pose['arm_flex_joint']),
            float(pose['arm_roll_joint']),
            float(pose['wrist_flex_joint']),
            float(pose['wrist_roll_joint']),
        ]
        self.pub_arm_goal.publish(arm)

        if include_head:
            head = Float32MultiArray()
            head.data = [
                float(pose['head_pan_joint']),
                float(pose['head_tilt_joint']),
            ]
            self.pub_head_goal_pose.publish(head)

        self.get_logger().info('NavModule.->No motion_synth: moving body to default_arm_pose')

    def send_nav_action_goal(self, goal: Pose2D, goal_distance: float = 0.0):
        if not self.nav_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavModule.->PumasNav action server not available')
            self._action_done = True
            self._action_success = False
            self._action_message = 'Action server unavailable'
            return

        goal_msg = PumasNav.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = Clock().now().to_msg()
        goal_pose.pose.position.x = float(goal.x)
        goal_pose.pose.position.y = float(goal.y)
        goal_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(goal.theta))
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        goal_msg.goal = goal_pose

        # Stop-short distance handled server-side by mvn_pln (<= 0 disables).
        goal_msg.goal_distance = (
            float(goal_distance) if goal_distance and goal_distance > 0 else 0.0
        )

        # TODO
        # goal_msg.patience = True

        arm_goal = StartAndEndJoints()
        arm_goal.has_arm_start_pose = False
        arm_goal.has_arm_end_pose = False

        if self.motion_synth_start_pose is not None:
            arm_goal.has_arm_start_pose = True
            arm_goal.start_pose = self.create_arm_joint_goal(
                self.motion_synth_start_pose)

        if self.motion_synth_end_pose is not None:
            arm_goal.has_arm_end_pose = True
            arm_goal.end_pose = self.create_arm_joint_goal(
                self.motion_synth_end_pose)

        arm_goal.motion_execution_time = float(self.motion_execution_time)
        goal_msg.arm_joints = arm_goal
        goal_msg.use_arm = arm_goal.has_arm_start_pose or arm_goal.has_arm_end_pose

        # Via points (empty -> mvn_pln plans a plain start->goal path).
        via_array = PoseArray()
        via_array.header.frame_id = 'map'
        via_array.header.stamp = Clock().now().to_msg()
        if self.via_points is not None:
            for vp in self.via_points:
                pose = Pose()
                pose.position.x = float(vp.x)
                pose.position.y = float(vp.y)
                pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, float(vp.theta))
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                via_array.poses.append(pose)
        goal_msg.via_points = via_array

        self.reset_action_state()
        self.marker_plot(goal_pose)

        self._send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback,
        )
        self._send_goal_future.add_done_callback(
            self.nav_goal_response_callback)

    def handle_robot_stop(self):
        self.pub_robot_stop.publish(Empty())

    def marker_plot(self, goal):
        self.marker.header.frame_id = 'map'
        self.marker.header.stamp = Clock().now().to_msg()
        self.marker.ns = 'goal_markers'
        self.marker.id = self.marker_num
        self.marker_num += 1
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose = goal.pose
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.pub_marker.publish(self.marker)

    def initialize_before_new_goal(self, send_stop=True):
        self.robot_stop = False

        if send_stop and self._goal_handle is not None:
            self.cancel_nav_action()
        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None
        self.motion_execution_time = 0.0
        self.via_points = None

        # Cancel any leftover gaze from a previous goal
        if self._gaze_active:
            self.cancel_gaze()
            self._set_move_head(True)

    def go_abs(
        self,
        goal: Pose2D,
        timeout,
        goal_distance=None,
        motion_synth=False,
        near_goal_callback=None,
    ) -> bool:
        self.get_logger().info(
            f'NavModule.->Go Absolute Goal(Action): x={goal.x}, y={goal.y}, theta={goal.theta}'
        )

        self.robot_stop = False
        self._near_goal_callback = near_goal_callback

        if not motion_synth:
            self.motion_synth_start_pose = None
            self.motion_synth_end_pose = None

        attempts = int(timeout * 10) if timeout != 0 else float('inf')

        # goal_distance (stop-short) is now honored server-side by mvn_pln: it
        # halts the robot and succeeds within goal_distance of the goal, so the
        # client only forwards the value and waits for the action result.
        self.send_nav_action_goal(
            goal,
            goal_distance=(goal_distance if goal_distance and goal_distance > 0 else 0.0),
        )

        if not self._external_node:
            executor = SingleThreadedExecutor()
            executor.add_node(self._node)
        else:
            executor = None

        result = False

        while rclpy.ok() and not self.robot_stop and attempts >= 0:
            if executor is not None:
                executor.spin_once(timeout_sec=0.1)

            if self._action_done:
                result = self._action_success
                break

            attempts -= 1

        if self.robot_stop:
            self.cancel_nav_action()
            result = False
        elif attempts < 0 and not self._action_done:
            self.get_logger().warn('NavModule.->Timeout waiting for PumasNav result')
            self.cancel_nav_action()
            result = False

        if not result:
            self.handle_robot_stop()

        return result

    def go_rel(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, timeout: float = 0) -> bool:
        """Relative holonomic move in the robot (base) frame.

        x: forward(+)/back(-) [m], y: left(+)/right(-) [m], yaw: rotate [rad].
        navlib only publishes [x, y, yaw] on /simple_move/goal_rel_pose; the
        motion is controlled by simple_move. Waits for /simple_move/goal_reached
        and returns True on SUCCEEDED (False on ABORTED/timeout/stop).
        """
        self.robot_stop = False
        self._move_done = False
        self._move_success = False

        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(yaw)]
        self.pub_goal_rel_pose.publish(msg)
        self.get_logger().info(
            f'NavModule.->go_rel: x={x:.3f} m, y={y:.3f} m, yaw={yaw:.3f} rad')

        attempts = int(timeout * 10) if timeout != 0 else float('inf')

        if not self._external_node:
            executor = SingleThreadedExecutor()
            executor.add_node(self._node)
        else:
            executor = None

        result = False
        while rclpy.ok() and not self.robot_stop and attempts >= 0:
            if executor is not None:
                executor.spin_once(timeout_sec=0.1)

            if self._move_done:
                result = self._move_success
                break

            attempts -= 1

        if attempts < 0 and not self._move_done:
            self.get_logger().warn('NavModule.->go_rel: timeout waiting for goal_reached')

        if self.robot_stop:
            # Stop simple_move via /navigation/stop (it subscribes there).
            self.handle_robot_stop()
            result = False

        return result

    def nav_goal(
        self,
        goal,
        timeout,
        motion_synth_pose=None,
        goal_distance=None,
        use_point_cloud=True,
        gaze_point=False,
        via_points=None,
        near_goal_callback=None,
    ):
        self.initialize_before_new_goal(send_stop=True)

        # Via points (list of Pose2D) the path should pass through in order.
        # None/empty -> normal start->goal planning.
        self.via_points = list(via_points) if via_points else None

        if motion_synth_pose is not None:
            self.get_logger().info('NavModule.->Motion Synth Nav Goal with Pose Config')
            self._set_use_point_cloud(False)

            if 'start' in motion_synth_pose:
                self.motion_synth_start_pose = motion_synth_pose['start']
            if 'goal' in motion_synth_pose:
                self.motion_synth_end_pose = motion_synth_pose['goal']
            self.motion_execution_time = float(
                motion_synth_pose.get('execution_time', 0.0))

        else:
            self.get_logger().info('NavModule.->Standard Nav Goal')
            if gaze_point is not False:
                self._set_use_point_cloud(False)
            else:
                self._set_use_point_cloud(use_point_cloud)
            self._send_default_arm_pose(include_head=(gaze_point is False))

        if gaze_point is not False:
            tf_name = gaze_point if isinstance(gaze_point, str) else ''
            restore_head = None
            if self.motion_synth_end_pose is not None:
                restore_head = (
                    self.motion_synth_end_pose.get('head_pan_joint', 0.0),
                    self.motion_synth_end_pose.get('head_tilt_joint', 0.0),
                )
            if self.start_gaze(tf_name, restore_head=restore_head):
                self._set_move_head(False)

        result = self.go_abs(
            goal,
            timeout,
            goal_distance,
            motion_synth=True,
            near_goal_callback=near_goal_callback,
        )

        # Ensure gaze is cancelled and move_head restored after nav ends
        if self._gaze_active:
            self.get_logger().info(
                'NavModule.->Nav ended: canceling gaze (fallback), restoring move_head'
            )
            self.cancel_gaze()
            self._set_move_head(True)

        return result


if __name__ == '__main__':
    rclpy.init()
    nav = NavModule()

    goal = Pose2D(x=1.0, y=3.7, theta=0.0)
    goal = Pose2D(x=0.8, y=3.44, theta=0.0)
    start_pose = {
        'arm_lift_joint': 0.0,
        'arm_flex_joint': np.deg2rad(0.0),
        'arm_roll_joint': np.deg2rad(0.0),
        'wrist_flex_joint': np.deg2rad(-90.0),
        'wrist_roll_joint': 0.0,
        'head_pan_joint': 0.0,
        'head_tilt_joint': np.deg2rad(0.0),
    }
    goal_pose = {
        'arm_lift_joint': 0.4,
        'arm_flex_joint': np.deg2rad(-90.0),
        'arm_roll_joint': np.deg2rad(0.0),
        'wrist_flex_joint': np.deg2rad(-90.0),
        'wrist_roll_joint': 0.0,
        'head_pan_joint': 0.0,
        'head_tilt_joint': np.deg2rad(60.0),
    }
    ms_config = {
        'start': start_pose,
        'goal': goal_pose,
        'execution_time': 0.5,
    }
    ms_config = None

    # for gaze node test
    gaze_tf = 'gaze_point'
    gaze_tf = False

    via_points = [
        Pose2D(x=2.6, y=3.9, theta=0.0),
        Pose2D(x=0.0, y=1.0, theta=0.0),
    ]
    via_points = None

    goal = Pose2D(x=0.5, y=3.6, theta=0.0)
    goal = Pose2D(x=0.0, y=0.0, theta=0.0)
    goal = Pose2D(x=2.58, y=2.0, theta=0.0)
    success = nav.nav_goal(
        goal,
        motion_synth_pose=ms_config,
        timeout=0,
        goal_distance=None,
        gaze_point=gaze_tf,
        via_points=via_points,
    )

    # test rel
    """
    ok = nav.go_rel(
        x=0.8,
        y=0.0,
        yaw=np.deg2rad(00),
        timeout=120,
    )
    """

    # for _ in range(10):
    #    success = nav.nav_goal(goal, motion_synth_pose=None, timeout=0, goal_distance=0)

    #    if success:
    #        nav.get_logger().info("NavStatus.->Nav Goal Reached")
    #    else:
    #        nav.get_logger().warn("NavStatus.->Failed to Reach Goal")

    #    time.sleep(2)
