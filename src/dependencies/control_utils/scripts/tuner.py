#!/usr/bin/env python3
"""
Generic ROS2 PID Tuner TUI
Subscribes to any PID controller's output + error topics and lets you tune
Kp, Ki, Kd via keyboard. Works with any PID_Controller node.

Usage:
    python3 pid_tuner.py --name yaw
    python3 pid_tuner.py --name depth --history 200

The node must expose:
    ROS2 params : <n>_pid_kp, <n>_pid_ki, <n>_pid_kd, <n>_pid_base_offset
    Publishers  : <n>_pid_output (std_msgs/Float32)
                  <n>_pid_error  (std_msgs/Float32)
    Service     : <n>_pid_reset  (std_srvs/Trigger)
"""

import argparse
import math
import random
import threading
import time
from collections import deque
from typing import Callable, Optional

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal
from textual.reactive import reactive
from textual.widgets import Footer, Header, Label, Static

try:
    import rclpy
    import rclpy.parameter
    from std_msgs.msg import Float32
    from std_srvs.srv import Trigger
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


# ─── Dual Plot Widget ─────────────────────────────────────────────────────────

BLOCKS = " ▁▂▃▄▅▆▇█"


class DualPlotWidget(Static):
    """
    Side-by-side scrolling block-character plots.
    Left = PID output,  Right = PID error.
    """

    DEFAULT_CSS = """
    DualPlotWidget {
        height: 100%;
        width: 100%;
        border: tall $primary;
        background: $surface;
        padding: 0 1;
    }
    """

    def __init__(self, history: int = 300, **kwargs):
        super().__init__(**kwargs)
        self.history = history
        self._output: deque[float] = deque(maxlen=history)
        self._error:  deque[float] = deque(maxlen=history)
        self._lock = threading.Lock()
        self._controller_name = ""

    def set_controller_name(self, name: str) -> None:
        self._controller_name = name

    def push_output(self, v: float) -> None:
        with self._lock:
            self._output.append(v)
        self.refresh()

    def push_error(self, v: float) -> None:
        with self._lock:
            self._error.append(v)
        self.refresh()

    @staticmethod
    def _render_series(
        data: list[float], width: int, height: int, label: str
    ) -> list[str]:
        if not data:
            rows = ["─" * width] * height
            mid = height // 2
            placeholder = f" {label} — no data "
            pad = max(0, (width - len(placeholder)) // 2)
            rows[mid] = (" " * pad + placeholder)[:width]
            return rows

        if len(data) > width:
            step = len(data) / width
            display = [data[int(i * step)] for i in range(width)]
        else:
            display = [0.0] * (width - len(data)) + data

        vmin = min(display)
        vmax = max(display)
        vrange = vmax - vmin or 1.0

        rows = []
        for row in range(height):
            line = ""
            for v in display:
                norm = (v - vmin) / vrange
                row_threshold = 1.0 - (row / height)
                block_idx = min(int(norm * 8), 8)
                line += BLOCKS[block_idx] if norm >= row_threshold else " "
            rows.append(line)

        header = f" {label}  [{vmin:.1f} … {vmax:.1f}] "
        rows[0] = header[:width].ljust(width)
        return rows

    def render(self) -> str:  # type: ignore[override]
        with self._lock:
            out_data = list(self._output)
            err_data = list(self._error)

        total_w = self.size.width - 2
        height  = self.size.height - 2

        if total_w <= 4 or height <= 2:
            return ""

        half = (total_w - 1) // 2

        out_rows = self._render_series(out_data, half,              height,
                                       f"{self._controller_name} output")
        err_rows = self._render_series(err_data, total_w - half - 1, height,
                                       f"{self._controller_name} error")

        return "\n".join(
            out_rows[r] + "│" + err_rows[r] for r in range(height)
        )


# ─── PID Param Panel ──────────────────────────────────────────────────────────

class ParamPanel(Static):

    DEFAULT_CSS = """
    ParamPanel {
        width: 1fr;
        height: 100%;
        border: tall $primary;
        padding: 1 2;
        content-align: center middle;
        layout: vertical;
    }
    ParamPanel.active {
        border: tall $accent;
        background: $accent 10%;
    }
    ParamPanel Label {
        width: 100%;
        content-align: center middle;
        text-align: center;
    }
    #param-title {
        text-style: bold;
        color: $text;
    }
    #param-value {
        text-style: bold;
        color: $accent;
    }
    #param-hint {
        color: $text-muted;
    }
    """

    value: reactive[float] = reactive(0.0)

    def __init__(self, param_name: str, **kwargs):
        super().__init__(**kwargs)
        self.param_name = param_name

    def compose(self) -> ComposeResult:
        yield Label(self.param_name, id="param-title")
        yield Label("0.0000",        id="param-value")
        yield Label("",              id="param-hint")

    def watch_value(self, v: float) -> None:
        try:
            self.query_one("#param-value", Label).update(f"{v:.4f}")
        except Exception:
            pass

    def set_hint(self, inc: str, dec: str) -> None:
        try:
            self.query_one("#param-hint", Label).update(
                f"[bold]{inc}[/] ▲   [bold]{dec}[/] ▼"
            )
        except Exception:
            pass

    def set_active(self, on: bool) -> None:
        (self.add_class if on else self.remove_class)("active")


# ─── ROS2 Bridge ──────────────────────────────────────────────────────────────

class ROSBridge:
    def __init__(
        self,
        name: str,
        on_output: Optional[Callable[[float], None]] = None,
        on_error:  Optional[Callable[[float], None]] = None,
    ):
        self.name      = name
        self.on_output = on_output
        self.on_error  = on_error
        self._node     = None
        self._running  = False
        self._thread: Optional[threading.Thread] = None

        self.kp          = 0.0
        self.ki          = 0.0
        self.kd          = 0.0
        self.base_offset = 1500.0

    def start(self) -> None:
        if not ROS_AVAILABLE:
            return
        rclpy.init(args=None)
        self._node = rclpy.create_node("pid_tuner_node")  # type: ignore

        for suffix, attr in [("kp", "kp"), ("ki", "ki"), ("kd", "kd"),
                              ("base_offset", "base_offset")]:
            try:
                v = self._node.get_parameter(f"{self.name}_pid_{suffix}").value
                setattr(self, attr, float(v))
            except Exception:
                pass

        self._node.create_subscription(
            Float32, f"{self.name}_pid_output", self._output_cb, 10)
        self._node.create_subscription(
            Float32, f"{self.name}_pid_error",  self._error_cb,  10)

        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._node:
            self._node.destroy_node()
        if ROS_AVAILABLE:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _spin(self) -> None:
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def _output_cb(self, msg: "Float32") -> None:
        if self.on_output:
            self.on_output(msg.data)

    def _error_cb(self, msg: "Float32") -> None:
        if self.on_error:
            self.on_error(msg.data)

    def set_param(self, attr: str, value: float) -> None:
        setattr(self, attr, value)
        if not ROS_AVAILABLE or not self._node:
            return
        self._node.set_parameters([
            rclpy.parameter.Parameter(
                f"{self.name}_pid_{attr}",
                rclpy.parameter.Parameter.Type.DOUBLE,
                value,
            )
        ])

    def call_reset(
        self, on_done: Optional[Callable[[bool, str], None]] = None
    ) -> None:
        """Call <name>_pid_reset (std_srvs/Trigger) asynchronously."""
        if not ROS_AVAILABLE or not self._node:
            if on_done:
                on_done(True, "demo mode — no-op")
            return

        client = self._node.create_client(Trigger, f"{self.name}_pid_reset")

        def _call() -> None:
            if not client.wait_for_service(timeout_sec=1.0):
                if on_done:
                    on_done(False, "service unavailable")
                return
            future = client.call_async(Trigger.Request())
            future.add_done_callback(
                lambda f: on_done and on_done(
                    f.result().success, f.result().message
                )
            )

        threading.Thread(target=_call, daemon=True).start()


# ─── Main TUI App ─────────────────────────────────────────────────────────────

STEP_LARGE = {"kp": 0.20, "ki": 0.005, "kd": 0.10}
STEP_SMALL = {"kp": 0.05, "ki": 0.001, "kd": 0.10}

KEY_BINDINGS_TABLE = [
    ("w / s", "Kp ▲ / ▼"),
    ("e / d", "Ki ▲ / ▼"),
    ("r / f", "Kd ▲ / ▼"),
    ("x",     "Reset integral"),
    ("q",     "Quit"),
]


class PIDTunerApp(App):
    """Generic ROS2 PID Tuner TUI."""

    CSS = """
    Screen {
        background: $background;
        layout: vertical;
    }
    #plot-container {
        height: 52%;
        width: 100%;
        padding: 0 1;
    }
    #param-row {
        height: 33%;
        width: 100%;
        padding: 0 1;
        layout: horizontal;
    }
    #keybind-bar {
        height: auto;
        width: 100%;
        padding: 0 1;
        layout: horizontal;
        background: $surface;
    }
    .kb-item {
        width: 1fr;
        content-align: center middle;
        padding: 0 1;
        color: $text-muted;
    }
    """

    BINDINGS = [
        Binding("w", "adjust('kp','up','large')", show=False),
        Binding("s", "adjust('kp','dn','small')", show=False),
        Binding("W", "adjust('kp','up','large')", show=False),
        Binding("S", "adjust('kp','dn','large')", show=False),
        Binding("e", "adjust('ki','up','large')", show=False),
        Binding("d", "adjust('ki','dn','small')", show=False),
        Binding("E", "adjust('ki','up','large')", show=False),
        Binding("D", "adjust('ki','dn','large')", show=False),
        Binding("r", "adjust('kd','up','large')", show=False),
        Binding("f", "adjust('kd','dn','small')", show=False),
        Binding("R", "adjust('kd','up','large')", show=False),
        Binding("F", "adjust('kd','dn','large')", show=False),
        Binding("x", "reset_integral",            show=False),
        Binding("q", "quit",                       show=False),
    ]

    def __init__(self, controller_name: str, history: int = 300):
        super().__init__()
        self.controller_name = controller_name
        self.history         = history
        self.bridge          = ROSBridge(
            controller_name,
            on_output=self._on_output,
            on_error =self._on_error,
        )
        self._sim_thread: Optional[threading.Thread] = None

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def on_mount(self) -> None:
        self.bridge.start()
        self._sync_panels()
        self.query_one("#plot", DualPlotWidget).set_controller_name(
            self.controller_name
        )

        if not ROS_AVAILABLE:
            self._sim_thread = threading.Thread(
                target=self._simulate, daemon=True)
            self._sim_thread.start()
            self.notify("ROS2 not found — demo mode", severity="warning")
        else:
            self.notify(
                f"Attached to '{self.controller_name}' PID controller",
                severity="information",
            )

    def on_unmount(self) -> None:
        self.bridge.stop()

    # ── composition ───────────────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        with Container(id="plot-container"):
            yield DualPlotWidget(history=self.history, id="plot")

        with Horizontal(id="param-row"):
            yield ParamPanel("Kp", id="panel-kp")
            yield ParamPanel("Ki", id="panel-ki")
            yield ParamPanel("Kd", id="panel-kd")

        with Horizontal(id="keybind-bar"):
            for keys, desc in KEY_BINDINGS_TABLE:
                yield Static(f"[bold cyan]{keys}[/]  {desc}", classes="kb-item")

        yield Footer()

    # ── helpers ───────────────────────────────────────────────────────────────

    def _sync_panels(self) -> None:
        for pid, attr, inc, dec in [
            ("panel-kp", "kp", "w", "s"),
            ("panel-ki", "ki", "e", "d"),
            ("panel-kd", "kd", "r", "f"),
        ]:
            p: ParamPanel = self.query_one(f"#{pid}", ParamPanel)
            p.value = getattr(self.bridge, attr)
            p.set_hint(inc, dec)

    def _on_output(self, v: float) -> None:
        self.call_from_thread(
            lambda: self.query_one("#plot", DualPlotWidget).push_output(v)
        )

    def _on_error(self, v: float) -> None:
        self.call_from_thread(
            lambda: self.query_one("#plot", DualPlotWidget).push_error(v)
        )

    def _simulate(self) -> None:
        t = 0.0
        while True:
            t += 0.05
            err = 45.0 * math.exp(-0.08 * t) * math.cos(1.5 * t)
            out = 1500 + self.bridge.kp * err + random.gauss(0, 1.5)
            self._on_output(out)
            self._on_error(err)
            time.sleep(0.05)

    # ── actions ───────────────────────────────────────────────────────────────

    def action_adjust(self, param: str, direction: str, size: str) -> None:
        step = (STEP_LARGE if size == "large" else STEP_SMALL)[param]
        if direction == "dn":
            step = -step

        new_val = round(getattr(self.bridge, param) + step, 6)
        self.bridge.set_param(param, new_val)

        pid = {"kp": "panel-kp", "ki": "panel-ki", "kd": "panel-kd"}[param]
        panel: ParamPanel = self.query_one(f"#{pid}", ParamPanel)
        panel.value = new_val
        panel.set_active(True)
        self.set_timer(0.3, lambda: panel.set_active(False))
        self.notify(f"{param.upper()} = {new_val:.4f}", timeout=1.5)

    def action_reset_integral(self) -> None:
        self.notify("Resetting integral…", severity="warning", timeout=1.0)

        def _done(ok: bool, msg: str) -> None:
            self.call_from_thread(
                lambda: self.notify(
                    f"Reset {'OK' if ok else 'FAILED'}: {msg}",
                    severity="information" if ok else "error",
                    timeout=2.0,
                )
            )

        self.bridge.call_reset(on_done=_done)

    def action_quit(self) -> None:
        self.exit()


# ─── Entry Point ──────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Generic ROS2 PID Tuner TUI")
    parser.add_argument("--name", "-n", default="yaw",
                        help="Controller name prefix (e.g. 'yaw', 'depth')")
    parser.add_argument("--history", "-H", type=int, default=300,
                        help="Plot history in samples (default: 300)")
    args = parser.parse_args()
    PIDTunerApp(controller_name=args.name, history=args.history).run()


if __name__ == "__main__":
    main()