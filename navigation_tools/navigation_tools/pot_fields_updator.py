#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from pumas_interfaces.srv import ParamReadWrite
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

_TARGET_NODE = 'potential_fields'
_TARGET_PARAMS = (
    'laser_pot_fields_d0',
    'laser_pot_fields_k_rej',
    # "cloud_pot_fields_d0",
    # "cloud_pot_fields_k_rej",
    'laser_max_x',
    # "cloud_max_x",
)
_RW_SERVICE = '/param_read_write'
_RW_CALL_TIMEOUT_SEC = 12.0
_RW_WAIT_SEC = 2.0

# Watchdog: if no True message is received for this long, fall back to
# navigation defaults. The publisher of /navigation/pot_fields_update is
# expected to send True at a steady rate while human-follow is active.
_WATCHDOG_PERIOD_SEC = 0.5
_TRUE_TIMEOUT_SEC = 1.5


class PotFieldsUpdator(Node):
    def __init__(self):
        super().__init__('pot_fields_updator')

        self._sub_cb_group = MutuallyExclusiveCallbackGroup()
        self._client_cb_group = MutuallyExclusiveCallbackGroup()

        # Fixed values used while following a human. Exposed as node parameters
        # so they can be tuned from the launch file without code edits.
        self.declare_parameter('human_follow.laser_pot_fields_d0', 0.50)
        self.declare_parameter('human_follow.laser_pot_fields_k_rej', 1.00)
        # self.declare_parameter("human_follow.cloud_pot_fields_d0",    0.50)
        # self.declare_parameter("human_follow.cloud_pot_fields_k_rej", 2.00)
        self.declare_parameter('human_follow.laser_max_x', 0.70)
        # self.declare_parameter("human_follow.cloud_max_x",            0.70)

        self._human_follow_values = {
            name: float(self.get_parameter(f'human_follow.{name}').value) for name in _TARGET_PARAMS
        }

        # navigation defaults are captured from the live potential_fields node
        # the first time we need them, so they reflect navigation.launch values.
        self._navigation_defaults = {}
        self._defaults_loaded = False
        self._defaults_lock = threading.Lock()

        # Current mode: 'navigation' (default) or 'human_follow'.
        self._mode = 'navigation'
        self._last_true_time = None

        self._rw_client = self.create_client(
            ParamReadWrite, _RW_SERVICE, callback_group=self._client_cb_group
        )

        self._sub = self.create_subscription(
            Bool,
            '/navigation/pot_fields_update',
            self.cb_update,
            10,
            callback_group=self._sub_cb_group,
        )

        self._watchdog_timer = self.create_timer(
            _WATCHDOG_PERIOD_SEC,
            self._watchdog,
            callback_group=self._sub_cb_group,
        )

        self.get_logger().info('PotFieldsUpdator.-> waiting for /navigation/pot_fields_update msg')

    def _call_rw(self, param_name: str, value: str = '', write: bool = False):
        if not self._rw_client.wait_for_service(timeout_sec=_RW_WAIT_SEC):
            self.get_logger().warn(
                f'PotFieldsUpdator.-> {_RW_SERVICE} not available')
            return None

        req = ParamReadWrite.Request()
        req.node_name = _TARGET_NODE
        req.param_name = param_name
        req.write = write
        req.value = value

        future = self._rw_client.call_async(req)

        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=_RW_CALL_TIMEOUT_SEC):
            self.get_logger().error(
                f'PotFieldsUpdator.-> {_RW_SERVICE} timeout for {param_name}')
            return None
        return future.result()

    def _load_navigation_defaults(self) -> bool:
        with self._defaults_lock:
            if self._defaults_loaded:
                return True

            snapshot = {}
            for name in _TARGET_PARAMS:
                res = self._call_rw(name, write=False)
                if res is None or not res.success:
                    self.get_logger().warn(
                        f'PotFieldsUpdator.-> Failed to read initial /{_TARGET_NODE}/{name}'
                    )
                    return False
                try:
                    snapshot[name] = float(res.param_value)
                except (TypeError, ValueError):
                    self.get_logger().warn(
                        f'PotFieldsUpdator.-> Unparseable value for {name}: {res.param_value}'
                    )
                    return False

            self._navigation_defaults = snapshot
            self._defaults_loaded = True
            self.get_logger().info(
                f'PotFieldsUpdator.-> Captured navigation defaults: {self._navigation_defaults}'
            )
            return True

    def _apply(self, label: str, values: dict):
        for name, value in values.items():
            res = self._call_rw(name, value=str(float(value)), write=True)
            if res is None or not res.success:
                self.get_logger().warn(
                    f'PotFieldsUpdator.-> Failed to write /{_TARGET_NODE}/{name} = {value}'
                )
                continue
            self.get_logger().info(
                f'PotFieldsUpdator.-> [{label}] /{_TARGET_NODE}/{name} = {value}'
            )

    def _switch_to(self, mode: str):
        if mode == 'human_follow':
            self._apply('human_follow', self._human_follow_values)
        else:
            self._apply('navigation', self._navigation_defaults)
        self._mode = mode

    def cb_update(self, msg: Bool):
        # Always make sure the navigation defaults are captured before the
        # first write, so a True-first message does not lose them.
        if not self._load_navigation_defaults():
            self.get_logger().error(
                'PotFieldsUpdator.-> Cannot proceed without navigation defaults'
            )
            return

        if msg.data:
            # Refresh watchdog regardless of mode.
            self._last_true_time = self.get_clock().now()
            if self._mode != 'human_follow':
                self.get_logger().warn(
                    'PotFieldsUpdator.-> Switch to human_follow parameters'
                )
                self._switch_to('human_follow')
        else:
            # Explicit False: revert immediately.
            if self._mode != 'navigation':
                self.get_logger().warn(
                    'PotFieldsUpdator.-> Restore navigation parameters (False received)'
                )
                self._switch_to('navigation')

    def _watchdog(self):
        # Only relevant while we are in human_follow mode.
        if self._mode != 'human_follow' or self._last_true_time is None:
            return

        elapsed_ns = (self.get_clock().now() - self._last_true_time).nanoseconds
        if elapsed_ns / 1e9 > _TRUE_TIMEOUT_SEC:
            self.get_logger().warn(
                f'PotFieldsUpdator.-> No True msg for >{_TRUE_TIMEOUT_SEC:.1f}s; '
                'restoring navigation parameters'
            )
            self._switch_to('navigation')


def main(args=None):
    rclpy.init(args=args)
    node = PotFieldsUpdator()
    executor = MultiThreadedExecutor(num_threads=2)
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
