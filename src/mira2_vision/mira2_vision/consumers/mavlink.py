"""Nonblocking, optional MAVLink transport for in-process sensor consumers."""
import os
import time
os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil


class SensorLink:
    def __init__(self, node, address, component, callback=None):
        self.node = node
        self.callback = callback
        self.last_heartbeat = None
        self.last_sent = 0.0
        self.connection = (mavutil.mavlink_connection(address, source_system=1,
                           source_component=component) if address else None)
        self.timer = node.create_timer(0.02, self.poll) if self.connection else None

    @property
    def ready(self):
        return (self.last_heartbeat is not None and
                time.monotonic() - self.last_heartbeat < 3.0)

    def poll(self):
        now = time.monotonic()
        if now - self.last_sent >= 1.0:
            # Also establishes an outbound UDP route before the first FCU heartbeat.
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                              mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_sent = now
        for _ in range(64):
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'HEARTBEAT' and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.connection.target_system = msg.get_srcSystem()
                self.connection.target_component = msg.get_srcComponent()
                self.last_heartbeat = time.monotonic()
            if self.ready and msg.get_srcSystem() == self.connection.target_system and self.callback:
                self.callback(msg)

    def close(self):
        if self.timer is not None:
            self.node.destroy_timer(self.timer)
        if self.connection is not None:
            self.connection.close()
