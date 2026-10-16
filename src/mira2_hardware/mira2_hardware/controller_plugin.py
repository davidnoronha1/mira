"""Controller plugin lifecycle and startup selection, independent of ROS."""

from importlib import import_module


class ControllerPlugin:
    """One capability using the controller's node, connection and safety state.

    Plugins never receive from or close the MAVLink connection. ``on_message``
    gets each accepted packet; ``tick`` runs at 50 Hz while connected. Stream
    requirements are merged by message ID, using the highest requested rate.
    Select plugins at startup; command plugins arbitrate using reported mode.
    """

    requires = ()
    message_rates = {}

    def __init__(self, controller):
        self.controller = controller

    def __getattr__(self, name):
        return getattr(self.controller, name)

    def on_connect(self):
        pass

    def on_message(self, msg):
        pass

    def tick(self, now):
        pass

    def stop(self, reason):
        pass


BUILTINS = {
    'rc': 'mira2_hardware.plugins.rc:RCControl',
    'guided': 'mira2_hardware.plugins.guided:GuidedControl',
    'navigation': 'mira2_hardware.plugins.navigation:NavigationFeedback',
    'telemetry': 'mira2_hardware.plugins.telemetry:TelemetryPublisher',
    'external_navigation': 'mira2_hardware.plugins.external_navigation:ExternalNavigation',
    'obstacle_distance': 'mira2_hardware.plugins.obstacle_distance:ObstacleDistance',
}


def resolve_plugins(names):
    """Validate all classes before opening hardware; allow module:Class plugins."""
    if len(names) != len(set(names)):
        raise ValueError('Duplicate controller plugins')
    classes = {}
    for name in names:
        spec = BUILTINS.get(name, name)
        module, separator, symbol = spec.partition(':')
        if not separator:
            raise ValueError(f'Unknown controller plugin: {name}')
        cls = getattr(import_module(module), symbol)
        if not isinstance(cls, type) or not issubclass(cls, ControllerPlugin):
            raise ValueError(f'{name} must inherit ControllerPlugin')
        classes[name] = cls
    ordered = {}
    pending = dict(classes)
    while pending:
        available = [name for name, cls in pending.items()
                     if all(dependency in ordered for dependency in cls.requires)]
        if not available:
            raise ValueError('Missing or cyclic controller plugin dependencies')
        for name in available:
            ordered[name] = pending.pop(name)
    return ordered
