#!/usr/bin/env python3
"""One ROS node and MAVLink connection, with startup-selected capabilities."""
import math
import os
import time
from types import SimpleNamespace

os.environ.setdefault('MAVLINK20', '1')

import rclpy
from pymavlink import mavutil
from rclpy.node import Node
from std_srvs.srv import SetBool
from utils.parameters import Setting, declare_config, positive, string_list

from mira2_hardware.config import DEFAULTS
from mira2_hardware.controller_plugin import resolve_plugins
from mira2_hardware.mavlink_io import MavlinkIO, autopilot_heartbeat, heartbeat_armed


class PixhawkController(MavlinkIO, Node):
    HEARTBEAT_TIMEOUT_SECONDS = 3.0
    TELEMETRY_TIMEOUT_SECONDS = 0.5

    def __init__(self, connection=None, **node_options):
        super().__init__('pixhawk_controller', **node_options)
        self.master = None
        self.plugins = {}
        self.ready = self.arm_state = self.requested_arm = self.emergency_active = False
        self.mode = None
        self.last_heartbeat = 0.0
        self.messages = {}
        self.message_times = {}
        self.last_stream_request = 0.0
        try:
            config = declare_config(self, {
                **DEFAULTS,
                'plugins': Setting(['rc', 'telemetry'], string_list, read_only=True, dynamic_typing=True),
                'simulation': False, 'auto_arm': False, 'visualize': False,
                'radius': Setting(0.3, positive),
                'direct_speed': Setting(0.8, positive),
                'direct_accel': Setting(0.3, positive),
                'path_max_points': Setting(2000, lambda value: type(value) is int and value > 0),
                'path_rate': Setting(2.0, positive),
            })
            self.options = SimpleNamespace(**config)
            if self.get_parameter('use_sim_time').value and not self.options.simulation:
                raise ValueError('use_sim_time requires simulation:=true')
            if not self.options.simulation:
                self.options.auto_arm = False
            classes = resolve_plugins(config['plugins'])
            for name, cls in classes.items():
                self.plugins[name] = cls(self)
            self.stream_rates = {mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT: 1.0}
            for plugin in self.plugins.values():
                for message_id, frequency in plugin.message_rates.items():
                    if not math.isfinite(frequency) or frequency <= 0:
                        raise ValueError(f'Invalid stream rate in {type(plugin).__name__}')
                    self.stream_rates[message_id] = max(self.stream_rates.get(message_id, 0), frequency)
            # Validate plugins and ROS interfaces before opening a serial device.
            self.master = connection or mavutil.mavlink_connection(
                config['pixhawk_address'], baud=config['baudrate'], source_system=253)
            self.create_service(SetBool, 'emergency_stop', self.set_emergency_stop)
            self.create_timer(0.02, self.tick)
            self.get_logger().info(f"Plugins: {', '.join(self.plugins) or '(none)'}; waiting for autopilot heartbeat")
        except Exception:
            if self.master is not None:
                try:
                    self.request_arm(False)
                finally:
                    self.master.close()
            self.destroy_node()
            raise

    def request_arm(self, armed):
        if armed and (not self.ready or self.emergency_active):
            return False
        super().request_arm(armed)
        self.requested_arm = bool(armed)
        return True

    def request_mode(self, custom_mode):
        if self.ready and not self.emergency_active:
            super().request_mode(custom_mode)

    def stop(self, reason):
        try:
            self.request_arm(False)
        finally:
            for plugin in self.plugins.values():
                try:
                    plugin.stop(reason)
                except Exception as exc:
                    self.get_logger().error(f'{type(plugin).__name__} stop failed: {exc}')
            self.get_logger().warn(reason)

    def set_emergency_stop(self, request, response):
        self.emergency_active = bool(request.data)
        self.stop('Emergency engaged' if request.data else 'Emergency cleared; remain disarmed')
        response.success = True
        response.message = 'Emergency engaged' if request.data else 'Emergency cleared; send a new command'
        return response

    def fresh_message(self, kind):
        if time.monotonic() - self.message_times.get(kind, 0.0) <= self.TELEMETRY_TIMEOUT_SECONDS:
            return self.messages.get(kind)
        return None

    def _request_streams(self, now):
        for message_id, frequency in self.stream_rates.items():
            self.request_message_interval(message_id, frequency)
        self.last_stream_request = now

    def _dispatch(self, msg, now):
        kind = msg.get_type()
        heartbeat = kind == 'HEARTBEAT'
        if heartbeat and not autopilot_heartbeat(msg):
            return
        if not self.ready:
            if not heartbeat:
                return
            self.master.target_system = msg.get_srcSystem()
            self.master.target_component = msg.get_srcComponent()
            self.messages.clear()
            self.message_times.clear()
            self.ready = True
            self.arm_state = heartbeat_armed(msg)
            self.mode = msg.custom_mode
            self.last_heartbeat = now
            if self.arm_state:
                self.stop('Autopilot connected while armed; requesting disarm')
            for plugin in self.plugins.values():
                plugin.on_connect()
            self._request_streams(now)
        if (msg.get_srcSystem() != self.master.target_system or
                msg.get_srcComponent() != self.master.target_component):
            return
        if heartbeat:
            self.last_heartbeat = now
            self.arm_state = heartbeat_armed(msg)
            self.mode = msg.custom_mode
        self.messages[kind] = msg
        self.message_times[kind] = now
        if (kind == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
                and msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED):
            self.get_logger().warn(f'Autopilot rejected a stream interval: result={msg.result}')
        for plugin in self.plugins.values():
            plugin.on_message(msg)

    def tick(self):
        try:
            for msg in self.receive_messages():
                self._dispatch(msg, time.monotonic())
            now = time.monotonic()
            if not self.ready:
                return
            if now - self.last_heartbeat > self.HEARTBEAT_TIMEOUT_SECONDS:
                self.ready = False
                self.stop('Autopilot heartbeat lost')
                return
            if now - self.last_stream_request >= 2.0:
                self._request_streams(now)
            for plugin in self.plugins.values():
                plugin.tick(now)
        except Exception:
            self.stop('Controller plugin or MAVLink processing failed')
            raise

    def close(self):
        try:
            self.stop('Controller shutdown')
        finally:
            self.master.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PixhawkController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
