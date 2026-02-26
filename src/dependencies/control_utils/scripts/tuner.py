#!/usr/bin/env python3
"""
Generic ROS2 PID Tuner TUI.

Launch via ros2 run:
    ros2 run control_utils pid_tuner --ros-args -p prefix:=yaw
    ros2 run control_utils pid_tuner --ros-args -p prefix:=depth

Or standalone demo (no ROS):
    python3 pid_tuner.py

The controller node is discovered automatically by inspecting who is
publishing on <prefix>_pid_output -- no need to pass a node name.

Expects the PID_Controller node to expose:
    Publishers : <prefix>_pid_output  (std_msgs/Float32)
                 <prefix>_pid_error   (std_msgs/Float32)
    Service    : <prefix>_pid_reset   (std_srvs/Trigger)
    Params     : <prefix>_pid_kp/ki/kd/base_offset
"""

import math
import random
import sys
import threading
import time
from collections import deque
from typing import Callable, Optional

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.reactive import reactive
from textual.widgets import Footer, Header, Label, Static
from textual_plotext import PlotextPlot

try:
    import rclpy
    import rclpy.parameter
    from rclpy.node import Node
    from std_msgs.msg import Float32
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


STEP_LARGE = {"kp": 0.20, "ki": 0.005, "kd": 0.10}
STEP_SMALL = {"kp": 0.05, "ki": 0.001, "kd": 0.10}

KEY_TABLE = [
    ("w / s", "Kp  large / small"),
    ("e / d", "Ki  large / small"),
    ("r / f", "Kd  large / small"),
    ("x",     "Reset integral"),
    ("q",     "Quit"),
]


# ---------------------------------------------------------------------------
# ROS2 Bridge
# ---------------------------------------------------------------------------

class ROSBridge:
    def __init__(
        self,
        prefix: str,
        on_output: Optional[Callable[[float], None]] = None,
        on_error:  Optional[Callable[[float], None]] = None,
        on_params_ready: Optional[Callable[[], None]] = None,
    ) -> None:
        self.prefix          = prefix
        self.on_output       = on_output
        self.on_error        = on_error
        self.on_params_ready = on_params_ready

        self.kp:          float = 0.0
        self.ki:          float = 0.0
        self.kd:          float = 0.0
        self.base_offset: float = 1500.0

        self._node: Optional["Node"] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._controller_node_name: Optional[str] = None

    # -- lifecycle ------------------------------------------------------------

    def start(self) -> None:
        if not ROS_AVAILABLE:
            return
        if not rclpy.ok():
            rclpy.init(args=sys.argv)

        self._node = rclpy.create_node("pid_tuner_tui")

        self._node.declare_parameter("prefix", self.prefix)
        self.prefix = str(
            self._node.get_parameter("prefix").get_parameter_value().string_value
        )

        self._node.create_subscription(
            Float32,
            f"{self.prefix}_pid_output",
            lambda m: self.on_output and self.on_output(m.data),
            10,
        )
        self._node.create_subscription(
            Float32,
            f"{self.prefix}_pid_error",
            lambda m: self.on_error and self.on_error(m.data),
            10,
        )

        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        # Discover controller node and fetch current param values in background.
        threading.Thread(target=self._discover_and_fetch, daemon=True).start()

    def stop(self) -> None:
        self._running = False
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if ROS_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _spin(self) -> None:
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.02)

    # -- discovery + initial param fetch -------------------------------------

    def _discover_and_fetch(self) -> None:
        """
        Poll get_publishers_info_by_topic() until we find the node publishing
        on <prefix>_pid_output, then call its GetParameters service to read
        the current kp/ki/kd values into the bridge.
        """
        topic    = f"/{self.prefix}_pid_output"
        deadline = time.time() + 10.0

        node_name: Optional[str] = None
        while time.time() < deadline and rclpy.ok():
            infos = self._node.get_publishers_info_by_topic(topic)
            if infos:
                ns   = infos[0].node_namespace.rstrip("/")
                name = infos[0].node_name
                node_name = f"{ns}/{name}" if ns else f"/{name}"
                break
            time.sleep(0.2)

        if node_name is None:
            self._node.get_logger().warn(
                f"No publisher found on '{topic}' after 10 s. "
                "Is the controller running?"
            )
            return

        self._controller_node_name = node_name
        self._node.get_logger().info(f"Controller node: {node_name}")

        # Fetch current param values via GetParameters service.
        client = self._node.create_client(
            GetParameters,
            f"{node_name}/get_parameters",
        )
        if not client.wait_for_service(timeout_sec=3.0):
            self._node.get_logger().warn("get_parameters service not available")
            return

        names = [
            f"{self.prefix}_pid_kp",
            f"{self.prefix}_pid_ki",
            f"{self.prefix}_pid_kd",
            f"{self.prefix}_pid_base_offset",
        ]
        future = client.call_async(GetParameters.Request(names=names))
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        if not future.done():
            return

        attrs = ["kp", "ki", "kd", "base_offset"]
        for attr, pv in zip(attrs, future.result().values):
            if pv.type == ParameterType.PARAMETER_DOUBLE:
                setattr(self, attr, pv.double_value)

        if self.on_params_ready:
            self.on_params_ready()

    # -- param setter --------------------------------------------------------

    def set_param(self, attr: str, value: float) -> None:
        """Send a parameter update to the controller node's set_parameters service."""
        setattr(self, attr, value)
        if not ROS_AVAILABLE or not self._node:
            return

        if self._controller_node_name is None:
            self._node.get_logger().warn(
                "Controller node not yet discovered -- skipping param set"
            )
            return

        client = self._node.create_client(
            SetParameters,
            f"{self._controller_node_name}/set_parameters",
        )

        pv = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=value,
        )
        p = Parameter(name=f"{self.prefix}_pid_{attr}", value=pv)

        def _worker() -> None:
            if not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().warn("set_parameters service not available")
                return
            future = client.call_async(SetParameters.Request(parameters=[p]))
            while rclpy.ok() and not future.done():
                time.sleep(0.01)
            if future.done():
                result = future.result().results[0]
                if not result.successful:
                    self._node.get_logger().warn(
                        f"set_parameters failed: {result.reason}"
                    )

        threading.Thread(target=_worker, daemon=True).start()

    # -- reset service -------------------------------------------------------

    def call_reset(
        self, on_done: Optional[Callable[[bool, str], None]] = None
    ) -> None:
        if not ROS_AVAILABLE or not self._node:
            if on_done:
                on_done(True, "demo mode")
            return

        client = self._node.create_client(Trigger, f"{self.prefix}_pid_reset")

        def _worker() -> None:
            if not client.wait_for_service(timeout_sec=1.0):
                if on_done:
                    on_done(False, "service not available")
                return
            future = client.call_async(Trigger.Request())
            while rclpy.ok() and not future.done():
                time.sleep(0.01)
            if future.done() and on_done:
                res = future.result()
                on_done(res.success, res.message)

        threading.Thread(target=_worker, daemon=True).start()


# ---------------------------------------------------------------------------
# Param Panel
# ---------------------------------------------------------------------------

class ParamPanel(Static):

    DEFAULT_CSS = """
    ParamPanel {
        width: 1fr;
        height: 100%;
        border: tall $primary;
        padding: 1 2;
        layout: vertical;
        content-align: center middle;
    }
    ParamPanel.active {
        border: tall $accent;
        background: $accent 10%;
    }
    ParamPanel Label {
        width: 100%;
        text-align: center;
        content-align: center middle;
    }
    #title { text-style: bold; }
    #value { text-style: bold; color: $accent; }
    #hint  { color: $text-muted; }
    """

    value: reactive[float] = reactive(0.0)

    def __init__(self, title: str, **kwargs) -> None:
        super().__init__(**kwargs)
        self._title = title

    def compose(self) -> ComposeResult:
        yield Label(self._title, id="title")
        yield Label("0.0000",   id="value")
        yield Label("",         id="hint")

    def watch_value(self, v: float) -> None:
        try:
            self.query_one("#value", Label).update(f"{v:.4f}")
        except Exception:
            pass

    def set_hint(self, inc: str, dec: str) -> None:
        try:
            self.query_one("#hint", Label).update(
                f"[bold]{inc}[/] up   [bold]{dec}[/] dn"
            )
        except Exception:
            pass

    def flash(self) -> None:
        self.add_class("active")
        self.app.set_timer(0.3, lambda: self.remove_class("active"))


# ---------------------------------------------------------------------------
# Live Plot
# ---------------------------------------------------------------------------

class LivePlot(PlotextPlot):

    DEFAULT_CSS = """
    LivePlot {
        height: 100%;
        width: 100%;
        border: tall $primary;
    }
    """

    def __init__(self, history: int = 200, prefix: str = "", **kwargs) -> None:
        super().__init__(**kwargs)
        self._history = history
        self._prefix  = prefix
        self._output: deque[float] = deque(maxlen=history)
        self._error:  deque[float] = deque(maxlen=history)
        self._lock = threading.Lock()

    def push_output(self, v: float) -> None:
        with self._lock:
            self._output.append(v)

    def push_error(self, v: float) -> None:
        with self._lock:
            self._error.append(v)

    def on_mount(self) -> None:
        self.set_interval(1 / 20, self._redraw)

    def _redraw(self) -> None:
        with self._lock:
            out = list(self._output)
            err = list(self._error)

        plt = self.plt
        plt.clear_figure()
        plt.theme("dark")
        plt.title(f"{self._prefix} PID  |  output (green)  error (red)")
        plt.xlabel("samples")

        n = max(len(out), len(err), 1)

        if out:
            plt.plot(list(range(n - len(out), n)), out,
                     color="green", label="output")
        if err:
            plt.plot(list(range(n - len(err), n)), err,
                     color="red", label="error")

        if not out and not err:
            plt.text("waiting for data...", x=0, y=0)

        self.refresh()


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

class PIDTunerApp(App):

    CSS = """
    Screen {
        layout: vertical;
        background: $background;
    }
    #plot-box {
        height: 55%;
        width: 100%;
        padding: 0 1;
    }
    #param-row {
        height: 30%;
        width: 100%;
        padding: 0 1;
        layout: horizontal;
    }
    #kb-bar {
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

    def __init__(self, prefix: str = "yaw", history: int = 200) -> None:
        super().__init__()
        self._prefix  = prefix
        self._history = history
        self.bridge   = ROSBridge(
            prefix          = prefix,
            on_output       = self._on_output,
            on_error        = self._on_error,
            on_params_ready = self._on_params_ready,
        )
        self._sim_thread: Optional[threading.Thread] = None

    # -- lifecycle ------------------------------------------------------------

    def on_mount(self) -> None:
        self.bridge.start()
        self._prefix = self.bridge.prefix
        self.title   = f"PID Tuner  [{self._prefix}]"
        self._sync_panels()

        if not ROS_AVAILABLE:
            self._sim_thread = threading.Thread(
                target=self._simulate, daemon=True)
            self._sim_thread.start()
            self.notify("rclpy not found -- demo mode", severity="warning")
        else:
            self.notify(
                f"Attached to '{self._prefix}' -- discovering controller node...",
                severity="information",
            )

    def on_unmount(self) -> None:
        self.bridge.stop()

    # -- composition ----------------------------------------------------------

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        with Vertical(id="plot-box"):
            yield LivePlot(history=self._history, prefix=self._prefix, id="plot")

        with Horizontal(id="param-row"):
            yield ParamPanel("Kp", id="panel-kp")
            yield ParamPanel("Ki", id="panel-ki")
            yield ParamPanel("Kd", id="panel-kd")

        with Horizontal(id="kb-bar"):
            for keys, desc in KEY_TABLE:
                yield Static(f"[bold cyan]{keys}[/]  {desc}", classes="kb-item")

        yield Footer()

    # -- data callbacks (called from ROS/sim thread) --------------------------

    def _on_output(self, v: float) -> None:
        try:
            self.query_one("#plot", LivePlot).push_output(v)
        except Exception:
            pass

    def _on_error(self, v: float) -> None:
        try:
            self.query_one("#plot", LivePlot).push_error(v)
        except Exception:
            pass

    def _on_params_ready(self) -> None:
        """Called from bg thread after initial param values are fetched."""
        self.call_from_thread(self._sync_panels)
        self.call_from_thread(
            lambda: self.notify(
                f"Controller: {self.bridge._controller_node_name}  "
                f"Kp={self.bridge.kp:.3f}  Ki={self.bridge.ki:.4f}  Kd={self.bridge.kd:.3f}",
                severity="information",
                timeout=4.0,
            )
        )

    # -- demo simulation ------------------------------------------------------

    def _simulate(self) -> None:
        t = 0.0
        while True:
            t += 0.05
            err = 45.0 * math.exp(-0.08 * t) * math.cos(1.5 * t)
            out = 1500.0 + self.bridge.kp * err + random.gauss(0, 1.5)
            self._on_output(out)
            self._on_error(err)
            time.sleep(0.05)

    # -- helpers --------------------------------------------------------------

    def _sync_panels(self) -> None:
        for pid, attr, inc, dec in [
            ("panel-kp", "kp", "w", "s"),
            ("panel-ki", "ki", "e", "d"),
            ("panel-kd", "kd", "r", "f"),
        ]:
            try:
                panel: ParamPanel = self.query_one(f"#{pid}", ParamPanel)
                panel.value = getattr(self.bridge, attr)
                panel.set_hint(inc, dec)
            except Exception:
                pass

    # -- actions --------------------------------------------------------------

    def action_adjust(self, param: str, direction: str, size: str) -> None:
        step = (STEP_LARGE if size == "large" else STEP_SMALL)[param]
        if direction == "dn":
            step = -step

        new_val = round(getattr(self.bridge, param) + step, 6)
        self.bridge.set_param(param, new_val)

        pid_map = {"kp": "panel-kp", "ki": "panel-ki", "kd": "panel-kd"}
        panel: ParamPanel = self.query_one(f"#{pid_map[param]}", ParamPanel)
        panel.value = new_val
        panel.flash()
        self.notify(f"{param.upper()} = {new_val:.4f}", timeout=1.5)

    def action_reset_integral(self) -> None:
        self.notify("Resetting integral...", severity="warning", timeout=1.0)

        def _done(ok: bool, msg: str) -> None:
            self.call_from_thread(
                lambda: self.notify(
                    f"Reset {'OK' if ok else 'FAILED'}: {msg}",
                    severity="information" if ok else "error",
                    timeout=2.5,
                )
            )

        self.bridge.call_reset(on_done=_done)

    def action_quit(self) -> None:
        self.exit()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--prefix", "-p", default="yaw")
    parser.add_argument("--history", "-H", type=int, default=200)
    known, _ = parser.parse_known_args()
    PIDTunerApp(prefix=known.prefix, history=known.history).run()


if __name__ == "__main__":
    main()