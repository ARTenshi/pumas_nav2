#!/usr/bin/env python3
# mainteiner ry0hei-kobayashi

import copy
import time

import numpy as np
import rclpy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Path
from pumas_interfaces.action import MotionSynthesis
from pumas_interfaces.msg import MotionPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion


class MotionSynth(Node):
    def __init__(self):
        super().__init__('motion_synth_server')

        # Action callback runs synchronously and blocks (time.sleep). It MUST
        # be on a separate callback group from the subscriptions so that under
        # MultiThreadedExecutor, /navigation/status and /simple_move/goal_path
        # keep arriving while execute_callback waits. The robot pose is now
        # pulled on-demand from TF inside the loop (see get_global_pose_from_tf).
        self._action_cb_group = MutuallyExclusiveCallbackGroup()
        self._sub_cb_group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            self,
            MotionSynthesis,
            '/motion_synth',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_cb_group,
        )

        # Single atomic message carrying Joints + motion_execution_time on a
        # dedicated topic. arm_controller and head_controller both subscribe
        # and pick up their respective fields. simple_move continues to use
        # the legacy Float32(MultiArray) topics unchanged.
        self.motion_pose_pub = self.create_publisher(
            MotionPose, '/hardware/motion_pose', 10)

        # Per-goal motion execution time. Set in execute_callback before any
        # send_pose call.
        self._motion_execution_time = 0.5

        self.current_pose = None
        self.global_nav_goal_reached = False
        self.path_received = False
        self.path_points = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            GoalStatus,
            '/navigation/status',
            self.navigation_status_callback,
            10,
            callback_group=self._sub_cb_group,
        )
        self.create_subscription(
            Path,
            '/simple_move/goal_path',
            self.path_callback,
            10,
            callback_group=self._sub_cb_group,
        )

        self.get_logger().info('motion_synth_server.-> is ready')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def path_callback(self, msg: Path):
        if not msg.poses:
            return
        self.path_points = [[p.pose.position.x, p.pose.position.y]
                            for p in msg.poses]
        self.path_received = True
        self.get_logger().info(
            f'motion_synth -> Received Path Length: {len(self.path_points)}')

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

            return x, y, yaw

        except TransformException as ex:
            self.get_logger().warn(
                f'motion_synth -> TF lookup failed: {target_frame} -> {source_frame}: {ex}'
            )
            return None

    def navigation_status_callback(self, msg):
        # actionlib_msgs/GoalStatus uses SUCCEEDED=3 (not STATUS_SUCCEEDED=4
        # from action_msgs). mvn_pln publishes actionlib_msgs on this topic.
        # Log every transition so we can see why global_nav_goal_reached
        # latches (or doesn't).
        if msg.status != getattr(self, '_last_nav_status', None):
            self.get_logger().info(
                f'motion_synth -> /navigation/status status={msg.status} '
                f"(SUCCEEDED=3, ACTIVE=1, ABORTED=4) text='{msg.text}'"
            )
            self._last_nav_status = msg.status
        if msg.status == GoalStatus.SUCCEEDED:
            self.global_nav_goal_reached = True

    def send_pose(self, joints):
        msg = MotionPose()
        msg.joints = joints
        msg.motion_execution_time = float(self._motion_execution_time)
        self.motion_pose_pub.publish(msg)

    # Risk classification tags returned by _classify_goal_pose_risk.
    RISK_LARGE_FORWARD_FLEX = 'large_forward_flex'
    RISK_SELF_COLLISION_ZONE = 'self_collision_zone'

    def _is_large_forward_flex(self, goal_pose):
        """Arm bent strongly forward (arm_flex < -1.0 rad ≈ -57°).

        Not a static self-collision, but we don't want the arm to swing all the
        way forward in one shot during navigation — it adds inertia and the
        wrist can swing into the base. Staged motion (partial fold first, full
        fold once positioned) keeps the swing tame.
        """
        return goal_pose.arm_flex_joint < -1.0

    def _is_self_collision_zone(self, goal_pose):
        """Arm near-vertical with the lift raised — true self-collision risk.

        At lift > 0.15 m the shoulder is high enough that an arm pointing
        near-straight-up (arm_flex > -0.35 rad ≈ -20°) hits the head/torso
        structure. Staged motion folds the arm forward before committing to
        this lift height.
        """
        return goal_pose.arm_flex_joint > -0.35 and goal_pose.arm_lift_joint > 0.15

    def _classify_goal_pose_risk(self, goal_pose):
        """Return the risk tag for the goal pose, or None if no staging needed."""
        if self._is_large_forward_flex(goal_pose):
            return self.RISK_LARGE_FORWARD_FLEX
        if self._is_self_collision_zone(goal_pose):
            return self.RISK_SELF_COLLISION_ZONE
        return None

    def create_temporary_pose(self, goal_pose):
        tmp = copy.deepcopy(goal_pose)
        if self._is_large_forward_flex(goal_pose) or self._is_self_collision_zone(goal_pose):
            tmp.arm_flex_joint = -0.6
        return tmp

    def joint_goal_reached(self, goal_joints, threshold=0.5):
        # TODO: Replace with real check
        return True

    @staticmethod
    def _distance_xy(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return (dx * dx + dy * dy) ** 0.5

    @staticmethod
    def _path_matches_goal(points, goal_location, tolerance=1.0):
        if not points:
            return False
        last = points[-1]
        dx = last[0] - goal_location.x
        dy = last[1] - goal_location.y
        return (dx * dx + dy * dy) ** 0.5 < tolerance

    # Trigger configuration. Tune in-place if real-robot behavior demands it.
    TRIGGER_WAYPOINT_FRACTION = 0.75  # snapshot waypoint at 75% along first path
    TRIGGER_WAYPOINT_RADIUS = 1.0  # m, distance to frozen waypoint to fire
    TRIGGER_GOAL_SAFETY_RADIUS = 0.6  # m, fallback when detour skipped waypoint
    MIN_SETTLE_SEC = 1.5  # min seconds after start_pose before trigger
    # Final goal pose is sent only after mvn_pln has finished the path and is
    # in its final-yaw correction phase. We detect that phase by requiring the
    # robot to be inside this radius around the goal location — during normal
    # navigation the yaw may transiently match goal_theta and we must NOT fire
    # on that false positive.
    FINAL_POSE_GOAL_RADIUS = 0.3  # m
    FINAL_POSE_YAW_TOLERANCE = 0.3  # rad
    # If the robot is already this close to the goal at execute time, the path
    # planner will return no/short path and the path-based trigger never fires.
    # Take the trivial-nav fast path instead.
    TRIVIAL_NAV_DISTANCE = 0.15  # m

    def _execute_trivial(self, goal, goal_handle, start_pose_dispatch_time):
        risk = self._classify_goal_pose_risk(goal.goal_pose)
        needs_staging = risk is not None

        # Let start_pose play out before we layer the next command on top.
        elapsed_sec = (self.get_clock().now() -
                       start_pose_dispatch_time).nanoseconds * 1e-9
        settle_remaining = max(0.0, self.MIN_SETTLE_SEC - elapsed_sec)
        if settle_remaining > 0:
            time.sleep(settle_remaining)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return MotionSynthesis.Result(result=False)

        if needs_staging:
            self.get_logger().info(
                'motion_synth -> trivial nav: sending staging pose'
            )
            self.send_pose(self.create_temporary_pose(goal.goal_pose))
            time.sleep(self.MIN_SETTLE_SEC)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MotionSynthesis.Result(result=False)

        self.get_logger().info(
            'motion_synth -> trivial nav: sending final goal pose'
        )
        self.send_pose(goal.goal_pose)
        # joint_goal_reached is a TODO stub that just returns True, so wait a
        # fixed settle period to give the hardware time to reach the pose.
        time.sleep(self.MIN_SETTLE_SEC)

        goal_handle.succeed()
        return MotionSynthesis.Result(result=True)

    def execute_callback(self, goal_handle):
        # NOTE: declared as a regular function (not `async`) on purpose. rclpy's
        # executor does not run an asyncio event loop, so `await asyncio.sleep`
        # raises RuntimeError. We use `time.sleep` here and rely on the
        # MultiThreadedExecutor + separate _sub_cb_group so subscriptions keep
        # delivering messages while we block.
        self.get_logger().info('Executing motion_synth goal...')
        goal = goal_handle.request
        feedback = MotionSynthesis.Feedback()

        # motion_execution_time == 0.0 means "unset"; fall back to 0.5 s.
        self._motion_execution_time = (
            float(goal.motion_execution_time)
            if goal.motion_execution_time > 0.0 else 0.5
        )
        self.get_logger().info(
            f'motion_synth -> motion_execution_time = '
            f'{self._motion_execution_time:.3f} s'
        )

        # Reset only the per-goal latching flag. Do NOT reset path_points,
        # because mvn_pln publishes the goal_path only once per replan and the
        # message may have arrived either before or after this callback starts.
        # Stale paths from a previous goal are filtered by _path_matches_goal.
        self.global_nav_goal_reached = False

        start_pose_dispatch_time = self.get_clock().now()
        if goal.apply_start_pose:
            self.send_pose(goal.start_pose)
            start_pose_dispatch_time = self.get_clock().now()

        # Trivial-nav fast path: when the robot is already at the goal, the
        # planner returns no/short path and the path-based trigger never fires,
        # so the goal_pose would be skipped. Detect this case and execute the
        # arm motion directly.
        if self.current_pose is not None:
            cur_xy = (self.current_pose[0], self.current_pose[1])
            goal_xy = (goal.goal_location.x, goal.goal_location.y)
            d_goal = self._distance_xy(cur_xy, goal_xy)
            if d_goal < self.TRIVIAL_NAV_DISTANCE:
                self.get_logger().info(
                    f'motion_synth -> trivial nav (d_goal={d_goal:.3f} m '
                    f'< {self.TRIVIAL_NAV_DISTANCE:.2f} m); '
                    'executing arm motion directly'
                )
                return self._execute_trivial(goal, goal_handle,
                                             start_pose_dispatch_time)

        # Wait until a path whose tail matches this goal_location arrives.
        # 15 s is longer than mvn_pln's 10 s potential-fields timeout so a slow
        # path-calculation chain still gets through.
        path_wait_iters = 150
        for _ in range(path_wait_iters):
            if self._path_matches_goal(self.path_points, goal.goal_location):
                break
            time.sleep(0.1)
        if not self._path_matches_goal(self.path_points, goal.goal_location):
            self.get_logger().warn('No path matching goal_location received, aborting.')
            goal_handle.abort()
            return MotionSynthesis.Result(result=False)

        # Snapshot the FIRST matching path and freeze a waypoint at 75% of it.
        # mvn_pln republishes a new goal_path on every replan/detour, which
        # would drift an index-on-latest-path approach; freezing the waypoint
        # in map frame makes the trigger geographically stable.
        first_path = list(self.path_points)
        if len(first_path) < 2:
            self.get_logger().warn(
                'Frozen path is too short to choose a trigger waypoint, aborting.'
            )
            goal_handle.abort()
            return MotionSynthesis.Result(result=False)

        trigger_idx = min(
            len(first_path) - 1,
            max(1, int(len(first_path) * self.TRIGGER_WAYPOINT_FRACTION)),
        )
        arm_trigger_xy = tuple(first_path[trigger_idx])
        goal_xy = (goal.goal_location.x, goal.goal_location.y)
        self.get_logger().info(
            f'motion_synth -> frozen trigger waypoint = '
            f'({arm_trigger_xy[0]:.2f}, {arm_trigger_xy[1]:.2f}) '
            f'(idx {trigger_idx}/{len(first_path)}), '
            f'goal safety radius = {self.TRIGGER_GOAL_SAFETY_RADIUS:.2f} m, '
            f'settle delay = {self.MIN_SETTLE_SEC:.2f} s'
        )

        triggered = False
        temporary_pose = None
        temporary_pose_sent = False
        final_pose_sent = False
        risk = self._classify_goal_pose_risk(goal.goal_pose)
        needs_staging = risk is not None
        if risk == self.RISK_LARGE_FORWARD_FLEX:
            self.get_logger().info(
                f'motion_synth -> large arm flex detected '
                f'(arm_flex={goal.goal_pose.arm_flex_joint:.2f} rad). '
                f'Staging motion via temporary pose.'
            )
            temporary_pose = self.create_temporary_pose(goal.goal_pose)
        elif risk == self.RISK_SELF_COLLISION_ZONE:
            self.get_logger().warn(
                f'motion_synth -> self-collision zone detected '
                f'(arm_flex={goal.goal_pose.arm_flex_joint:.2f} rad, '
                f'arm_lift={goal.goal_pose.arm_lift_joint:.2f} m). '
                f'Using temporary pose.'
            )
            temporary_pose = self.create_temporary_pose(goal.goal_pose)

        time.sleep(0.1)

        # Log loop state every LOG_PERIOD iterations (≈1 s at 0.05 s sleep).
        LOG_PERIOD = 20
        loop_iter = 0

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                goal_handle.canceled()
                return MotionSynthesis.Result(result=False)

            pose_from_tf = self.get_global_pose_from_tf()
            if pose_from_tf is not None:
                self.current_pose = pose_from_tf

            if not triggered and self.current_pose:
                elapsed_sec = (self.get_clock().now() -
                               start_pose_dispatch_time).nanoseconds * 1e-9
                cur_xy = (self.current_pose[0], self.current_pose[1])
                dist_to_waypoint = self._distance_xy(cur_xy, arm_trigger_xy)
                dist_to_goal = self._distance_xy(cur_xy, goal_xy)
                if loop_iter % LOG_PERIOD == 0:
                    self.get_logger().info(
                        f'motion_synth -> waiting trigger: '
                        f'elapsed={elapsed_sec:.2f}/{self.MIN_SETTLE_SEC:.2f} s, '
                        f'd_wp={dist_to_waypoint:.2f}/{self.TRIGGER_WAYPOINT_RADIUS:.2f} m, '
                        f'd_goal={dist_to_goal:.2f}/{self.TRIGGER_GOAL_SAFETY_RADIUS:.2f} m'
                    )
                if elapsed_sec >= self.MIN_SETTLE_SEC:
                    fire = False
                    reason = ''
                    if dist_to_waypoint < self.TRIGGER_WAYPOINT_RADIUS:
                        fire = True
                        reason = (
                            f'dist_to_waypoint={dist_to_waypoint:.2f} m < '
                            f'{self.TRIGGER_WAYPOINT_RADIUS:.2f} m'
                        )
                    elif dist_to_goal < self.TRIGGER_GOAL_SAFETY_RADIUS:
                        fire = True
                        reason = (
                            f'safety: dist_to_goal={dist_to_goal:.2f} m < '
                            f'{self.TRIGGER_GOAL_SAFETY_RADIUS:.2f} m'
                        )

                    if fire:
                        self.get_logger().info(
                            f'Triggering arm motion ({reason})')
                        if needs_staging:
                            self.get_logger().info(
                                'motion_synth -> sending temporary pose, '
                                'will wait for yaw alignment or nav SUCCEEDED'
                            )
                            self.send_pose(temporary_pose)
                            temporary_pose_sent = True
                        else:
                            self.get_logger().info(
                                'motion_synth -> no staging needed, sending '
                                'final goal pose directly'
                            )
                            self.send_pose(goal.goal_pose)
                            final_pose_sent = True
                        triggered = True

            if triggered:
                if temporary_pose_sent and not final_pose_sent:
                    yaw_error = abs(
                        self.current_pose[2] - goal.goal_location.theta)
                    if yaw_error > np.pi:
                        yaw_error = 2 * np.pi - yaw_error
                    cur_xy = (self.current_pose[0], self.current_pose[1])
                    dist_to_goal = self._distance_xy(cur_xy, goal_xy)
                    yaw_aligned_at_goal = (
                        yaw_error < self.FINAL_POSE_YAW_TOLERANCE and
                        dist_to_goal < self.FINAL_POSE_GOAL_RADIUS
                    )
                    if loop_iter % LOG_PERIOD == 0:
                        self.get_logger().info(
                            f'motion_synth -> waiting final: '
                            f'd_goal={dist_to_goal:.2f}/'
                            f'{self.FINAL_POSE_GOAL_RADIUS:.2f} m, '
                            f'yaw_error={yaw_error:.2f}/'
                            f'{self.FINAL_POSE_YAW_TOLERANCE:.2f} rad '
                            f'(both required), '
                            f'nav_goal_reached={self.global_nav_goal_reached}'
                        )
                    if yaw_aligned_at_goal or self.global_nav_goal_reached:
                        fire_reason = (
                            'yaw aligned at goal' if yaw_aligned_at_goal
                            else 'nav SUCCEEDED'
                        )
                        self.get_logger().info(
                            f'motion_synth -> sending final goal pose '
                            f'({fire_reason}, '
                            f'd_goal={dist_to_goal:.2f} m, '
                            f'yaw_error={yaw_error:.2f} rad, '
                            f'nav_goal_reached={self.global_nav_goal_reached})'
                        )
                        self.send_pose(goal.goal_pose)
                        final_pose_sent = True
                        self.global_nav_goal_reached = False

            if triggered and final_pose_sent:
                if self.joint_goal_reached(goal.goal_pose):
                    self.get_logger().info('Final arm pose reached.')
                    goal_handle.succeed()
                    return MotionSynthesis.Result(result=True)

            goal_handle.publish_feedback(feedback)
            loop_iter += 1
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = MotionSynth()
    # MultiThreadedExecutor is required because execute_callback blocks on
    # time.sleep; subscriptions live on a different callback group so they
    # continue to be served while the action callback waits.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
