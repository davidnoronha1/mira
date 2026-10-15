#!/usr/bin/env python3
"""MAVLink master with bounded nonblocking receive polling."""
from mira2_hardware.master_core import MasterCore, run


class AsyncPixhawkMaster(MasterCore):
    def __init__(self, connection=None):
        super().__init__(connection=connection, node_name="async_master")

    def receive_messages(self):
        for _ in range(self.RECEIVE_BUDGET):
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            yield msg


# Retain the familiar class name for tools injecting a MAVLink connection.
PixhawkMaster = AsyncPixhawkMaster


def main(args=None):
    run(AsyncPixhawkMaster, args)


if __name__ == "__main__":
    main()
