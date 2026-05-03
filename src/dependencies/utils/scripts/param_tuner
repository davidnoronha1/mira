#!/usr/bin/env python3
"""
ROS2 Parameter Browser - A Textual TUI for viewing and editing ROS2 node parameters.

Usage:
    python3 ros2_param_browser.py

Requirements:
    pip install textual
    ROS2 environment must be sourced.
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters, DescribeParameters

from textual.app import App, ComposeResult
from textual.widgets import (
    Header, Footer, Tree, Label, Input, Button,
    Static, DataTable, TabbedContent, TabPane,
)
from textual.containers import Horizontal, Vertical
from textual.screen import ModalScreen
from textual.binding import Binding
from textual import work
from textual.reactive import reactive
import yaml
from rclpy.qos import QoSProfile
import json
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ─────────────────────────── helpers ────────────────────────────

PARAM_TYPE_NAMES = {
    ParameterType.PARAMETER_NOT_SET:        "NOT_SET",
    ParameterType.PARAMETER_BOOL:           "bool",
    ParameterType.PARAMETER_INTEGER:        "int",
    ParameterType.PARAMETER_DOUBLE:         "float",
    ParameterType.PARAMETER_STRING:         "str",
    ParameterType.PARAMETER_BYTE_ARRAY:     "byte[]",
    ParameterType.PARAMETER_BOOL_ARRAY:     "bool[]",
    ParameterType.PARAMETER_INTEGER_ARRAY:  "int[]",
    ParameterType.PARAMETER_DOUBLE_ARRAY:   "float[]",
    ParameterType.PARAMETER_STRING_ARRAY:   "str[]",
}


def param_value_str(pval) -> str:
    t = pval.type
    if t == ParameterType.PARAMETER_NOT_SET:        return "<not set>"
    if t == ParameterType.PARAMETER_BOOL:           return str(pval.bool_value)
    if t == ParameterType.PARAMETER_INTEGER:        return str(pval.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:         return str(pval.double_value)
    if t == ParameterType.PARAMETER_STRING:         return pval.string_value
    if t == ParameterType.PARAMETER_BYTE_ARRAY:     return str(list(pval.byte_array_value))
    if t == ParameterType.PARAMETER_BOOL_ARRAY:     return str(list(pval.bool_array_value))
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:  return str(list(pval.integer_array_value))
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:   return str(list(pval.double_array_value))
    if t == ParameterType.PARAMETER_STRING_ARRAY:   return str(list(pval.string_array_value))
    return "?"


def parse_value_for_type(raw: str, ptype: int):
    raw = raw.strip()
    if ptype == ParameterType.PARAMETER_BOOL:
        if raw.lower() in ("true",  "1", "yes"):  return True
        if raw.lower() in ("false", "0", "no"):   return False
        raise ValueError(f"Cannot parse '{raw}' as bool")
    if ptype == ParameterType.PARAMETER_INTEGER:       return int(raw)
    if ptype == ParameterType.PARAMETER_DOUBLE:        return float(raw)
    if ptype == ParameterType.PARAMETER_STRING:        return raw
    if ptype in (ParameterType.PARAMETER_INTEGER_ARRAY,
                 ParameterType.PARAMETER_DOUBLE_ARRAY,
                 ParameterType.PARAMETER_BOOL_ARRAY,
                 ParameterType.PARAMETER_STRING_ARRAY):
        import ast
        lst = ast.literal_eval(raw)
        if ptype == ParameterType.PARAMETER_INTEGER_ARRAY:  return [int(x)   for x in lst]
        if ptype == ParameterType.PARAMETER_DOUBLE_ARRAY:   return [float(x) for x in lst]
        if ptype == ParameterType.PARAMETER_BOOL_ARRAY:     return [bool(x)  for x in lst]
        return list(lst)
    raise ValueError(f"Unsupported type {ptype} for editing")


PTYPE_TO_RCLPY = {
    ParameterType.PARAMETER_BOOL:           Parameter.Type.BOOL,
    ParameterType.PARAMETER_INTEGER:        Parameter.Type.INTEGER,
    ParameterType.PARAMETER_DOUBLE:         Parameter.Type.DOUBLE,
    ParameterType.PARAMETER_STRING:         Parameter.Type.STRING,
    ParameterType.PARAMETER_BYTE_ARRAY:     Parameter.Type.BYTE_ARRAY,
    ParameterType.PARAMETER_BOOL_ARRAY:     Parameter.Type.BOOL_ARRAY,
    ParameterType.PARAMETER_INTEGER_ARRAY:  Parameter.Type.INTEGER_ARRAY,
    ParameterType.PARAMETER_DOUBLE_ARRAY:   Parameter.Type.DOUBLE_ARRAY,
    ParameterType.PARAMETER_STRING_ARRAY:   Parameter.Type.STRING_ARRAY,
}


# ───────────────────────── ROS2 bridge ──────────────────────────

class Ros2Bridge(Node):
    def __init__(self):
        super().__init__("ros2_param_browser")

    def get_node_names(self) -> list[str]:
        result = []
        for name, ns in self.get_node_names_and_namespaces():
            if name == "ros2_param_browser":
                continue
            full = (ns.rstrip("/") + "/" + name) if ns != "/" else ("/" + name)
            result.append(full)
        return sorted(result)

    def _call_srv(self, srv_type, endpoint, req, timeout=3.0):
        client = self.create_client(srv_type, endpoint)
        if not client.wait_for_service(timeout_sec=1.0):
            client.destroy()
            return None
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        client.destroy()
        return future.result()

    def list_params(self, node: str) -> list[str]:
        req = ListParameters.Request()
        req.depth = 0
        res = self._call_srv(ListParameters, f"{node}/list_parameters", req)
        return sorted(res.result.names) if res else []

    def get_params(self, node: str, names: list[str]):
        if not names:
            return []
        req = GetParameters.Request()
        req.names = names
        res = self._call_srv(GetParameters, f"{node}/get_parameters", req)
        return res.values if res else []

    def describe_params(self, node: str, names: list[str]):
        if not names:
            return []
        req = DescribeParameters.Request()
        req.names = names
        res = self._call_srv(DescribeParameters, f"{node}/describe_parameters", req)
        return res.descriptors if res else []

    def get_services_for_node(self, node_full: str) -> list[tuple[str, str]]:
        parts = node_full.rsplit("/", 1)
        ns    = parts[0] if len(parts) == 2 and parts[0] else "/"
        name  = parts[1] if len(parts) == 2 else parts[0].lstrip("/")
        try:
            svcs = self.get_service_names_and_types_by_node(name, ns)
        except Exception:
            svcs = []
        return sorted((s, ", ".join(t)) for s, t in svcs)

    def get_topics_for_node(self, node_full: str) -> tuple[list[tuple[str, str]], list[tuple[str, str]]]:
        """Returns (publishers, subscribers) for a node"""
        parts = node_full.rsplit("/", 1)
        ns    = parts[0] if len(parts) == 2 and parts[0] else "/"
        name  = parts[1] if len(parts) == 2 else parts[0].lstrip("/")
        try:
            pubs = self.get_publisher_names_and_types_by_node(name, ns)
            subs = self.get_subscriber_names_and_types_by_node(name, ns)
            pubs_list = sorted((t, ", ".join(types)) for t, types in pubs)
            subs_list = sorted((t, ", ".join(types)) for t, types in subs)
            return pubs_list, subs_list
        except Exception:
            return [], []

    def call_trigger_service(self, service_name: str) -> tuple[bool, str]:
        """Call a Trigger service (no request needed)"""
        from std_srvs.srv import Trigger
        client = self.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=2.0):
            client.destroy()
            return False, "Service not available"
        
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        client.destroy()
        
        if future.result() is not None:
            result = future.result()
            return result.success, result.message
        else:
            return False, "Service call failed or timed out"

    def set_param(self, node: str, name: str, value, ptype: int) -> str:
        rclpy_type = PTYPE_TO_RCLPY.get(ptype)
        if rclpy_type is None:
            return f"Editing type {ptype} not supported"
        param = Parameter(name, rclpy_type, value).to_parameter_msg()
        req = SetParameters.Request()
        req.parameters = [param]
        res = self._call_srv(SetParameters, f"{node}/set_parameters", req)
        if res is None:
            return "No response from node"
        r = res.results[0]
        return "" if r.successful else (r.reason or "Unknown error")

    def echo_topic(self, topic_name: str, topic_type: str, callback):
        """Subscribe to a topic and call callback with each message"""
        try:
            # Parse the message type
            msg_class = get_message(topic_type)
            
            # Create subscription
            subscription = self.create_subscription(
                msg_class,
                topic_name,
                lambda msg: callback(self._msg_to_dict(msg)),
                10
            )
            return subscription
        except Exception as e:
            callback({"error": str(e), "topic": topic_name, "type": topic_type})
            return None
    
    def _msg_to_dict(self, msg) -> dict:
        """Convert ROS message to dictionary for JSON serialization"""
        result = {}
        try:
            for field in msg.get_fields_and_field_types():
                value = getattr(msg, field)
                if hasattr(value, 'get_fields_and_field_types'):
                    # Nested message
                    result[field] = self._msg_to_dict(value)
                elif isinstance(value, (list, tuple)):
                    # Array - check if it contains messages
                    if value and hasattr(value[0], 'get_fields_and_field_types'):
                        result[field] = [self._msg_to_dict(v) for v in value]
                    else:
                        result[field] = list(value)
                else:
                    result[field] = value
        except Exception as e:
            result["_conversion_error"] = str(e)
        return result


# ────────────────────────── Edit Modal ──────────────────────────

class EditParamModal(ModalScreen):
    BINDINGS = [Binding("escape", "dismiss(None)", "Cancel")]

    CSS = """
    EditParamModal > Vertical {
        width: 72;
        height: auto;
        background: $surface;
        border: round $accent;
        padding: 1 2;
        margin: 4 0;
    }
    EditParamModal Label  { margin-bottom: 1; }
    EditParamModal Input  { margin-bottom: 1; }
    EditParamModal #btn_row { height: 3; }
    EditParamModal Button { width: 1fr; }
    """

    def __init__(self, node_name, param_name, current_value, ptype, description=""):
        super().__init__()
        self._node_name    = node_name
        self._param_name   = param_name
        self._current      = current_value
        self._ptype        = ptype
        self._description  = description

    def compose(self) -> ComposeResult:
        type_str = PARAM_TYPE_NAMES.get(self._ptype, "unknown")
        with Vertical():
            yield Label("[bold]Edit Parameter[/bold]")
            yield Label(f"Node:  [cyan]{self._node_name}[/cyan]")
            yield Label(f"Param: [yellow]{self._param_name}[/yellow]   "
                        f"Type: [green]{type_str}[/green]")
            if self._description:
                yield Label(f"[dim]{self._description}[/dim]")
            yield Input(value=self._current, id="vi", placeholder="New value…")
            with Horizontal(id="btn_row"):
                yield Button("Set",    variant="primary", id="btn_set")
                yield Button("Cancel", variant="default", id="btn_cancel")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "btn_cancel":
            self.dismiss(None)
        else:
            self.dismiss((self.query_one("#vi", Input).value, self._ptype))

    def on_input_submitted(self, _) -> None:
        self.dismiss((self.query_one("#vi", Input).value, self._ptype))


# ────────────────────────── Service Call Modal ─────────────────────────

class CallServiceModal(ModalScreen):
    BINDINGS = [Binding("escape", "dismiss(None)", "Cancel")]

    CSS = """
    CallServiceModal > Vertical {
        width: 72;
        height: auto;
        background: $surface;
        border: round $accent;
        padding: 1 2;
        margin: 4 0;
    }
    CallServiceModal Label  { margin-bottom: 1; }
    CallServiceModal #btn_row { height: 3; }
    CallServiceModal Button { width: 1fr; }
    CallServiceModal #result { 
        margin-top: 1;
        padding: 1;
        border: round $primary;
        min-height: 3;
    }
    """

    def __init__(self, service_name: str, service_type: str, bridge: Ros2Bridge):
        super().__init__()
        self._service_name = service_name
        self._service_type = service_type
        self._bridge = bridge

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Label("[bold]Call Service[/bold]")
            yield Label(f"Service: [cyan]{self._service_name}[/cyan]")
            yield Label(f"Type: [green]{self._service_type}[/green]")
            
            if "Trigger" in self._service_type:
                yield Label("[dim]This is a Trigger service (no parameters needed)[/dim]")
            
            yield Static("", id="result")
            
            with Horizontal(id="btn_row"):
                yield Button("Call",   variant="primary", id="btn_call")
                yield Button("Cancel", variant="default", id="btn_cancel")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "btn_cancel":
            self.dismiss(None)
        elif event.button.id == "btn_call":
            self._call_service()

    @work(thread=True)
    def _call_service(self) -> None:
        self.app.call_from_thread(
            self.query_one("#result", Static).update,
            "[yellow]Calling service...[/yellow]"
        )
        
        if "Trigger" in self._service_type:
            success, message = self._bridge.call_trigger_service(self._service_name)
            if success:
                result_text = f"[green]✓ Success[/green]\n{message}"
            else:
                result_text = f"[red]✗ Failed[/red]\n{message}"
        else:
            result_text = "[yellow]Only Trigger services are currently supported[/yellow]"
        
        self.app.call_from_thread(
            self.query_one("#result", Static).update,
            result_text
        )


# ────────────────────────── Topic Echo Modal ──────────────────────────

class EchoTopicModal(ModalScreen):
    BINDINGS = [Binding("escape", "dismiss(None)", "Close")]

    CSS = """
    EchoTopicModal > Vertical {
        width: 90%;
        height: 90%;
        background: $surface;
        border: round $accent;
        padding: 1 2;
    }
    EchoTopicModal Label { margin-bottom: 1; }
    EchoTopicModal #echo_log {
        height: 1fr;
        border: round $primary;
        background: $panel;
        padding: 1;
    }
    EchoTopicModal #btn_row { height: 3; margin-top: 1; }
    EchoTopicModal Button { width: 1fr; }
    """

    def __init__(self, topic_name: str, topic_type: str, bridge: Ros2Bridge):
        super().__init__()
        self._topic_name = topic_name
        self._topic_type = topic_type
        self._bridge = bridge
        self._subscription = None
        self._message_count = 0

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Label(f"[bold]Echo Topic[/bold]")
            yield Label(f"Topic: [cyan]{self._topic_name}[/cyan]")
            yield Label(f"Type: [green]{self._topic_type}[/green]")
            yield Label(f"Messages: [yellow]0[/yellow]", id="msg_count")
            from textual.widgets import RichLog
            yield RichLog(id="echo_log", highlight=True, markup=False)
            with Horizontal(id="btn_row"):
                yield Button("Clear", variant="default", id="btn_clear")
                yield Button("Close", variant="primary", id="btn_close")

    def on_mount(self) -> None:
        self._start_echo()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "btn_close":
            self._stop_echo()
            self.dismiss(None)
        elif event.button.id == "btn_clear":
            log = self.query_one("#echo_log", RichLog)
            log.clear()
            self._message_count = 0
            self.query_one("#msg_count", Label).update(f"Messages: [yellow]{self._message_count}[/yellow]")

    def _start_echo(self) -> None:
        def message_callback(msg_dict):
            self._message_count += 1
            try:
                compact_json = json.dumps(msg_dict, separators=(',', ':'))
            except Exception as e:
                compact_json = f'{{"_error": "Failed to serialize: {e}"}}'
            self.app.call_from_thread(self._add_message, compact_json)
        
        try:
            self._subscription = self._bridge.echo_topic(
                self._topic_name,
                self._topic_type,
                message_callback
            )
            if self._subscription:
                # Add initial message directly (we're on main thread)
                self._add_message(f"[Subscribed to {self._topic_name}]")
            else:
                self._add_message("[ERROR: Failed to create subscription]")
        except Exception as e:
            self._add_message(f"[ERROR: {e}]")

    def _add_message(self, json_str: str) -> None:
        from textual.widgets import RichLog
        try:
            log = self.query_one("#echo_log", RichLog)
            log.write(json_str)
            self.query_one("#msg_count", Label).update(f"Messages: [yellow]{self._message_count}[/yellow]")
        except Exception as e:
            # Widget might not be ready yet
            pass

    def _stop_echo(self) -> None:
        if self._subscription:
            self._bridge.destroy_subscription(self._subscription)
            self._subscription = None

    def on_unmount(self) -> None:
        self._stop_echo()


# ─────────────────────────── Main App ───────────────────────────

KEYBINDS_TEXT = (
    "  [bold]R[/bold]/[bold]F5[/bold] Refresh nodes   "
    "[bold]E[/bold] Edit param   "
    "[bold]C[/bold] Call service   "
    "[bold]X[/bold] Echo topic   "
    "[bold]Tab[/bold] Switch tab   "
    "[bold]1[/bold] Parameters   "
    "[bold]2[/bold] Services   "
    "[bold]3[/bold] Topics   "
    "[bold]↑↓[/bold] Navigate rows   "
    "[bold]Q[/bold] Quit"
)


class ParamBrowserApp(App):
    TITLE     = "ROS2 Parameter Browser"
    SUB_TITLE = "Browse & edit node parameters · ROS2 Python API"

    CSS = """
    Screen { layout: horizontal; }

    #left_pane {
        width: 32;
        border: round $accent;
        padding: 0 1;
    }
    #left_pane Label { padding: 0 1; }

    #right_pane { width: 1fr; layout: vertical; }

    TabbedContent { height: 1fr; }
    TabPane       { height: 1fr; padding: 0; }
    DataTable     { height: 1fr; }

    #status_bar {
        height: 3;
        border: round $accent;
        padding: 0 1;
        content-align: left middle;
    }

    #keybinds_bar {
        height: 3;
        border: round $primary;
        padding: 0 1;
        content-align: left middle;
        background: $boost;
        color: $text-muted;
    }
    """

    BINDINGS = [
        Binding("r",   "refresh_nodes", "Refresh", show=False),
        Binding("f5",  "refresh_nodes", "Refresh", show=False),
        Binding("e",   "edit_selected", "Edit",    show=False),
        Binding("c",   "call_service",  "Call",    show=False),
        Binding("x",   "echo_topic",    "Echo",    show=False),
        Binding("tab", "switch_tab",    "Tab",     show=False),
        Binding("1",   "goto_params",   "Params",  show=False),
        Binding("2",   "goto_services", "Services",show=False),
        Binding("3",   "goto_topics",   "Topics",  show=False),
        Binding("q",   "quit",          "Quit",    show=False),
    ]

    selected_node: reactive[str | None] = reactive(None)

    def __init__(self):
        super().__init__()
        rclpy.init()
        self._bridge        = Ros2Bridge()
        self._params_cache: dict[str, list[tuple]] = {}
        self._ros_thread = threading.Thread(target=lambda: rclpy.spin(self._bridge), daemon=True)
        self._ros_thread.start()

    # ── layout ────────────────────────────────────────────────

    def compose(self) -> ComposeResult:
        yield Header()
        with Horizontal():
            with Vertical(id="left_pane"):
                yield Label("[bold]Nodes[/bold]")
                yield Tree("ROS2 Nodes", id="node_tree")

            with Vertical(id="right_pane"):
                with TabbedContent(id="tabs"):
                    with TabPane("① Parameters", id="tab_params"):
                        yield DataTable(id="param_table", zebra_stripes=True, cursor_type="row")
                    with TabPane("② Services", id="tab_services"):
                        yield DataTable(id="svc_table", zebra_stripes=True, cursor_type="row")
                    with TabPane("③ Topics", id="tab_topics"):
                        with Vertical():
                            yield Label("[bold]Publishers[/bold]")
                            yield DataTable(id="pub_table", zebra_stripes=True, cursor_type="row")
                            yield Label("[bold]Subscribers[/bold]")
                            yield DataTable(id="sub_table", zebra_stripes=True, cursor_type="row")

                yield Static("Select a node from the tree to begin.", id="status_bar")
                yield Static(KEYBINDS_TEXT, id="keybinds_bar")
        yield Footer()

    def on_mount(self) -> None:
        self.query_one("#param_table", DataTable).add_columns(
            "Parameter Name", "Type", "Value", "Description"
        )
        self.query_one("#svc_table", DataTable).add_columns(
            "Service Name", "Type"
        )
        self.query_one("#pub_table", DataTable).add_columns(
            "Topic Name", "Type"
        )
        self.query_one("#sub_table", DataTable).add_columns(
            "Topic Name", "Type"
        )
        self.action_refresh_nodes()

    # ── node tree ─────────────────────────────────────────────

    @work(thread=True)
    def action_refresh_nodes(self) -> None:
        self._update_status("Refreshing node list…")
        nodes = self._bridge.get_node_names()
        self.call_from_thread(self._populate_tree, nodes)

    def _populate_tree(self, nodes: list[str]) -> None:
        tree = self.query_one("#node_tree", Tree)
        tree.clear()
        tree.root.expand()
        for n in nodes:
            tree.root.add_leaf(n, data=n)
        self._update_status(f"Found {len(nodes)} node(s). Select one to inspect.")

    def on_tree_node_selected(self, event: Tree.NodeSelected) -> None:
        node_name = event.node.data
        if node_name is None:
            return
        self.selected_node = node_name
        self._load_params(node_name)
        self._load_services(node_name)
        self._load_topics(node_name)

    # ── parameters ────────────────────────────────────────────

    @work(thread=True)
    def _load_params(self, node: str) -> None:
        self._update_status(f"Loading parameters for {node}…")
        names       = self._bridge.list_params(node)
        values      = self._bridge.get_params(node, names)
        descriptors = self._bridge.describe_params(node, names)
        rows = []
        for i, name in enumerate(names):
            pval  = values[i]      if i < len(values)      else None
            desc  = descriptors[i] if i < len(descriptors) else None
            ptype = pval.type      if pval                  else ParameterType.PARAMETER_NOT_SET
            vstr  = param_value_str(pval) if pval else "?"
            dstr  = (desc.description if desc and desc.description else "")
            rows.append((name, ptype, vstr, dstr))
        self._params_cache[node] = rows
        self.call_from_thread(self._show_params, node, rows)

    def _show_params(self, node: str, rows: list) -> None:
        t = self.query_one("#param_table", DataTable)
        t.clear()
        for name, ptype, vstr, dstr in rows:
            t.add_row(name, PARAM_TYPE_NAMES.get(ptype, "?"), vstr, dstr, key=name)
        self._update_status(
            f"[cyan]{node}[/cyan] — {len(rows)} param(s)  "
            f"[dim]Press [bold]E[/bold] to edit selected row[/dim]"
        )

    # ── services ──────────────────────────────────────────────

    @work(thread=True)
    def _load_services(self, node: str) -> None:
        svcs = self._bridge.get_services_for_node(node)
        self.call_from_thread(self._show_services, svcs)

    def _show_services(self, svcs: list[tuple[str, str]]) -> None:
        t = self.query_one("#svc_table", DataTable)
        t.clear()
        for svc_name, svc_type in svcs:
            t.add_row(svc_name, svc_type)

    # ── topics ────────────────────────────────────────────────

    @work(thread=True)
    def _load_topics(self, node: str) -> None:
        pubs, subs = self._bridge.get_topics_for_node(node)
        self.call_from_thread(self._show_topics, pubs, subs)

    def _show_topics(self, pubs: list[tuple[str, str]], subs: list[tuple[str, str]]) -> None:
        pub_table = self.query_one("#pub_table", DataTable)
        sub_table = self.query_one("#sub_table", DataTable)
        
        pub_table.clear()
        for topic_name, topic_type in pubs:
            pub_table.add_row(topic_name, topic_type)
        
        sub_table.clear()
        for topic_name, topic_type in subs:
            sub_table.add_row(topic_name, topic_type)

    # ── tab switching ─────────────────────────────────────────

    def action_switch_tab(self) -> None:
        tabs = self.query_one("#tabs", TabbedContent)
        current = tabs.active
        if current == "tab_params":
            tabs.active = "tab_services"
        elif current == "tab_services":
            tabs.active = "tab_topics"
        else:
            tabs.active = "tab_params"

    def action_goto_params(self) -> None:
        self.query_one("#tabs", TabbedContent).active = "tab_params"

    def action_goto_services(self) -> None:
        self.query_one("#tabs", TabbedContent).active = "tab_services"

    def action_goto_topics(self) -> None:
        self.query_one("#tabs", TabbedContent).active = "tab_topics"

    # ── editing ───────────────────────────────────────────────

    def action_edit_selected(self) -> None:
        if self.query_one("#tabs", TabbedContent).active != "tab_params":
            self._update_status("[yellow]Switch to the Parameters tab (press 1) to edit.[/yellow]")
            return
        node = self.selected_node
        if node is None:
            self._update_status("[red]No node selected.[/red]")
            return
        table = self.query_one("#param_table", DataTable)
        if table.cursor_row < 0:
            self._update_status("[red]No parameter row selected.[/red]")
            return
        row_key    = table.coordinate_to_cell_key((table.cursor_row, 0)).row_key
        param_name = str(row_key.value) if row_key.value else None
        if not param_name:
            return
        entry = next((r for r in self._params_cache.get(node, []) if r[0] == param_name), None)
        if entry is None:
            return
        _, ptype, current_val, dstr = entry
        if ptype in (ParameterType.PARAMETER_NOT_SET, ParameterType.PARAMETER_BYTE_ARRAY):
            self._update_status(
                f"[yellow]Type '{PARAM_TYPE_NAMES.get(ptype)}' cannot be edited.[/yellow]"
            )
            return
        self.push_screen(
            EditParamModal(node, param_name, current_val, ptype, dstr),
            callback=lambda res: self._handle_edit_result(node, param_name, res),
        )

    def _handle_edit_result(self, node, param_name, result) -> None:
        if result is None:
            return
        raw, ptype = result
        self._apply_param(node, param_name, raw, ptype)

    @work(thread=True)
    def _apply_param(self, node, param_name, raw, ptype) -> None:
        try:
            value = parse_value_for_type(raw, ptype)
        except Exception as exc:
            self.call_from_thread(self._update_status, f"[red]Parse error: {exc}[/red]")
            return
        error = self._bridge.set_param(node, param_name, value, ptype)
        if error:
            self.call_from_thread(self._update_status, f"[red]Set failed: {error}[/red]")
        else:
            self.call_from_thread(
                self._update_status,
                f"[green]✓ Set [bold]{param_name}[/bold] = {raw!r}[/green]",
            )
            self._load_params(node)

    # ── service calling ───────────────────────────────────────

    def action_call_service(self) -> None:
        if self.query_one("#tabs", TabbedContent).active != "tab_services":
            self._update_status("[yellow]Switch to the Services tab (press 2) to call.[/yellow]")
            return
        
        node = self.selected_node
        if node is None:
            self._update_status("[red]No node selected.[/red]")
            return
        
        table = self.query_one("#svc_table", DataTable)
        if table.cursor_row < 0:
            self._update_status("[red]No service row selected.[/red]")
            return
        
        row = table.get_row_at(table.cursor_row)
        service_name = row[0]
        service_type = row[1]
        
        self.push_screen(CallServiceModal(service_name, service_type, self._bridge))

    # ── topic echoing ─────────────────────────────────────────

    def action_echo_topic(self) -> None:
        if self.query_one("#tabs", TabbedContent).active != "tab_topics":
            self._update_status("[yellow]Switch to the Topics tab (press 3) to echo.[/yellow]")
            return
        
        node = self.selected_node
        if node is None:
            self._update_status("[red]No node selected.[/red]")
            return
        
        # Try to get selected topic from either pub or sub table
        pub_table = self.query_one("#pub_table", DataTable)
        sub_table = self.query_one("#sub_table", DataTable)
        
        topic_name = None
        topic_type = None
        
        # Check which table has focus
        if pub_table.has_focus and pub_table.cursor_row >= 0:
            row = pub_table.get_row_at(pub_table.cursor_row)
            topic_name = row[0]
            topic_type = row[1]
        elif sub_table.has_focus and sub_table.cursor_row >= 0:
            row = sub_table.get_row_at(sub_table.cursor_row)
            topic_name = row[0]
            topic_type = row[1]
        elif pub_table.cursor_row >= 0:
            # Default to pub table if no focus
            row = pub_table.get_row_at(pub_table.cursor_row)
            topic_name = row[0]
            topic_type = row[1]
        elif sub_table.cursor_row >= 0:
            row = sub_table.get_row_at(sub_table.cursor_row)
            topic_name = row[0]
            topic_type = row[1]
        
        if not topic_name:
            self._update_status("[red]No topic selected.[/red]")
            return
        
        # Handle comma-separated types (take first one)
        if ',' in topic_type:
            topic_type = topic_type.split(',')[0].strip()
        
        self.push_screen(EchoTopicModal(topic_name, topic_type, self._bridge))

    # ── helpers ───────────────────────────────────────────────

    def _update_status(self, msg: str) -> None:
        try:
            self.query_one("#status_bar", Static).update(msg)
        except Exception:
            pass

    def on_unmount(self) -> None:
        self._bridge.destroy_node()
        rclpy.shutdown()


# ─────────────────────────────── main ───────────────────────────

def main():
    ParamBrowserApp().run()