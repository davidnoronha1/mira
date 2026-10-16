"""Public contract for in-process detector and consumer extensions."""
from dataclasses import dataclass, field
from typing import Any
import importlib
from importlib.metadata import entry_points


@dataclass
class Frame:
    image: Any
    header: Any
    matrix: Any
    preview: Any = None
    products: dict = field(default_factory=dict)
    depth: Any = None


class Plugin:
    """Plugins share images/products by reference. Never modify Frame.image.

    Set requires/provides to product keys. Consumers run only when every required
    product was produced for this frame (rate-limited detectors never reuse data).
    Construction may declare plugin-specific ROS parameters through node.
    """
    requires = ()
    provides = ()
    after = ()

    def __init__(self, node):
        self.node = node
        self.settings = node.settings

    def process(self, frame):
        raise NotImplementedError

    def on_depth(self, depth):
        """Receive each fresh metric depth sample independently of RGB/inference."""
        pass

    def close(self):
        pass


def load_plugin(spec, node):
    builtins = {
        'obstacle_distance': 'mira2_vision.consumers.obstacle_distance:ObstacleDistance',
        'optical_flow': 'mira2_vision.consumers.optical_flow:OpticalFlow',
        'yolo': 'mira2_vision.detectors.yolo:Yolo',
        'gazebo_boxes': 'mira2_vision.detectors.gazebo_boxes:GazeboBoxes',
        'aruco': 'mira2_vision.detectors.aruco:Aruco',
        'pipeline': 'mira2_vision.detectors.pipeline:Pipeline',
        'objects': 'mira2_vision.consumers.objects:Objects',
        'depth_targets': 'mira2_vision.consumers.depth_targets:DepthTargets',
        'markers': 'mira2_vision.consumers.markers:Markers',
        'docking_board': 'mira2_vision.consumers.docking_board:DockingBoard',
    }
    target = builtins.get(spec, spec)
    if ':' in target:
        module, attribute = target.split(':', 1)
        factory = getattr(importlib.import_module(module), attribute)
    else:
        matches = [entry for entry in entry_points(group='mira2_vision.plugins') if entry.name == spec]
        if len(matches) != 1:
            raise ValueError(f'Unknown or ambiguous vision plugin: {spec}')
        factory = matches[0].load()
    return factory(node)


def order_plugins(plugins):
    """Validate dependencies and sort without silently enabling extra plugins."""
    pending, ordered, available = list(plugins), [], set()
    provided = [key for plugin in pending for key in plugin.provides]
    if len(provided) != len(set(provided)):
        raise ValueError('Vision plugins must have unique product providers')
    while pending:
        for plugin in pending:
            dependencies = set(plugin.requires) | (set(plugin.after) & set(provided))
            if dependencies <= available:
                ordered.append(plugin)
                available.update(plugin.provides)
                pending.remove(plugin)
                break
        else:
            missing = {type(p).__name__: sorted((set(p.requires) | (set(p.after) & set(provided))) - available)
                       for p in pending}
            raise ValueError(f'Missing or cyclic vision plugin dependencies: {missing}')
    return ordered
