"""Shared ROS parameter declarations; schemas and defaults belong to callers.

ROS resolves parameter files, node selectors and CLI overrides. These helpers
only declare defaults and validate the resulting values. Returned dictionaries
are startup snapshots; mutable parameters still need the node's update callback.
"""

from collections.abc import Mapping
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class Setting:
    """A default with optional startup validation and ROS descriptor policy."""

    default: object
    validate: object = None
    read_only: object = None
    dynamic_typing: bool = False


def positive(value):
    """Require a finite, strictly positive number."""
    return not isinstance(value, bool) and math.isfinite(value) and value > 0


def nonnegative(value):
    """Require a finite, nonnegative number."""
    return not isinstance(value, bool) and math.isfinite(value) and value >= 0


def choices(*values):
    """Build an enumeration validator."""
    return lambda value: value in values


def string_list(value):
    """Accept empty plugin selections as well as lists of string names."""
    return isinstance(value, list) and all(isinstance(item, str) for item in value)


def string_list_argument(value):
    """Parse a launch selection as a comma-separated list or YAML array."""
    import yaml

    value = value.strip()
    selected = (yaml.safe_load(value) if value.startswith("[") else
                [item.strip() for item in value.split(",") if item.strip()])
    if not string_list(selected):
        raise ValueError("Expected a list of string names")
    return selected


def flatten(values, namespace=""):
    """Expand nested mappings into dotted ROS names, rejecting collisions."""
    result = {}

    def visit(mapping, prefix):
        for key, value in mapping.items():
            name = f"{prefix}.{key}" if prefix else key
            if isinstance(value, Mapping):
                visit(value, name)
            else:
                if name in result:
                    raise ValueError(f"Duplicate parameter: {name}")
                result[name] = value

    visit(values, namespace)
    return result


def parameter(node, name, default, *, read_only=False):
    """Declare once, honor ROS overrides, and validate without coercing values.

    Existing declarations are read and type-checked, never redeclared or reset.
    Invalid overrides raise errors instead of silently falling back to defaults.
    """
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.parameter import Parameter

    setting = default if isinstance(default, Setting) else Setting(default)
    if setting.default is None:
        raise ValueError(f"{name}: an explicit ROS-compatible default is required")
    descriptor = ParameterDescriptor(
        read_only=read_only if setting.read_only is None else setting.read_only,
        dynamic_typing=setting.dynamic_typing)
    resolved = (node.get_parameter(name) if node.has_parameter(name) else
                node.declare_parameter(name, setting.default, descriptor))
    expected = Parameter.Type.from_parameter_value(setting.default)
    if not setting.dynamic_typing and resolved.type_ != expected:
        raise ValueError(f"{name}: expected {expected.name}, got {resolved.type_.name}")
    value = resolved.value
    if value is None and setting.dynamic_typing and isinstance(setting.default, list):
        # ROS cannot type an empty YAML array (e.g. detectors: []), so the
        # override arrives unset; an absent parameter would have the default.
        value = []
    if value is None:
        raise ValueError(f"{name}: parameter is not set")
    if setting.validate is not None:
        try:
            valid = setting.validate(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError(f"{name}: invalid value {value!r}: {error}") from error
        if not valid:
            raise ValueError(f"{name}: invalid value {value!r}")
    return value


def declare_config(node, defaults, *, namespace="", read_only=True, mutable=()):
    """Declare a local schema and return values keyed relative to namespace.

    Leaves may be raw defaults or Setting objects. Mutable names use dotted
    paths relative to this schema. Always use full names when declaring so ROS
    overrides also work for parameter groups on older ROS distributions.
    """
    values = flatten(defaults)
    unknown = set(mutable) - values.keys()
    if unknown:
        raise ValueError(f"Unknown mutable parameters: {sorted(unknown)}")
    return {name: parameter(node, f"{namespace}.{name}" if namespace else name,
                            value, read_only=read_only and name not in mutable)
            for name, value in values.items()}


def dataclass_config(node, defaults, *, namespace="", read_only=True, validators=None):
    """Load a flat dataclass schema, retaining its constructor validation."""
    values = asdict(defaults)
    for name, validate in (validators or {}).items():
        if name not in values:
            raise ValueError(f"Unknown parameter validator: {name}")
        values[name] = Setting(values[name], validate)
    return type(defaults)(**declare_config(
        node, values, namespace=namespace, read_only=read_only))
