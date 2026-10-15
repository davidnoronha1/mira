#!/usr/bin/env python3
"""Config-driven pipeline following using the shared normalized PID helper."""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from custom_msgs.msg import CommandsNormalized, Telemetry
from utils.control import PIDController
from utils.parameters import Setting, declare_config, positive


class PipelineControlsNode(Node):
    def __init__(self):
        super().__init__("pipeline_controls")
        defaults = {"control_rate": Setting(20.0, positive), "feedback_timeout": Setting(0.5, positive),
                    "forward_effort": 0.3, "output_limit": 0.3,
                    "lateral_tolerance": Setting(0.05, positive), "yaw_tolerance": Setting(5.0, positive),
                    "lateral.kp": 0.0, "lateral.ki": 0.0, "lateral.kd": 0.0,
                    "yaw.kp": 0.0, "yaw.ki": 0.0, "yaw.kd": 0.0,
                    "flight_mode": "ALT_HOLD"}
        self.settings = declare_config(self, defaults)
        if not 0 <= self.settings["forward_effort"] <= 1:
            raise ValueError("forward_effort must be in [0, 1]")
        self.pids = {axis: PIDController(
            **{key: self.settings[f"{axis}.{key}"] for key in ("kp", "ki", "kd")},
            limit=self.settings["output_limit"]) for axis in ("lateral", "yaw")}
        for pid in self.pids.values():
            pid.control(0.0, 0.01)
            pid.reset()
        self.enabled = False
        self.offset = self.angle = None
        self.offset_time = self.angle_time = self.telemetry_time = 0.0
        self.last_tick = time.monotonic()
        self.publisher = self.create_publisher(CommandsNormalized, "mira/command", 1)
        self.create_subscription(Point, "mira/perception/bottom/pipeline/offset", self.on_offset, 1)
        self.create_subscription(Float32, "mira/perception/bottom/pipeline/angle", self.on_angle, 1)
        self.create_subscription(Telemetry, "mira/telemetry", self.on_telemetry, 1)
        self.create_service(SetBool, "mira/control/pipeline/enable", self.set_enabled)
        self.create_timer(1 / self.settings["control_rate"], self.tick)

    def on_offset(self, msg):
        self.offset, self.offset_time = msg.x, time.monotonic()

    def on_angle(self, msg):
        self.angle, self.angle_time = msg.data, time.monotonic()

    def on_telemetry(self, msg):
        age = self.get_clock().now().nanoseconds / 1e9 - msg.timestamp
        if 0 <= age <= self.settings["feedback_timeout"]:
            self.telemetry_time = time.monotonic()

    def set_enabled(self, request, response):
        self.enabled = request.data
        for pid in self.pids.values():
            pid.reset()
        response.success = True
        response.message = "Pipeline enabled" if request.data else "Pipeline disabled"
        return response

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        timeout = self.settings["feedback_timeout"]
        fresh = self.offset is not None and self.angle is not None and \
            math.isfinite(self.offset) and math.isfinite(self.angle) and \
            all(now - stamp <= timeout for stamp in
                (self.offset_time, self.angle_time, self.telemetry_time)) and dt <= timeout
        if not fresh:
            self.enabled = False
        command = CommandsNormalized(arm=self.enabled, mode=self.settings["flight_mode"])
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        if self.enabled:
            command.yaw = self.pids["yaw"].control(self.angle / 90.0, dt)
            command.lateral = self.pids["lateral"].control(-self.offset, dt)
            if abs(self.angle) <= self.settings["yaw_tolerance"] and abs(self.offset) <= self.settings["lateral_tolerance"]:
                command.forward = self.settings["forward_effort"]
        else:
            for pid in self.pids.values():
                pid.reset()
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineControlsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        command = CommandsNormalized(mode=node.settings["flight_mode"])
        command.header.stamp = node.get_clock().now().to_msg()
        command.header.frame_id = "base_link"
        node.publisher.publish(command)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
