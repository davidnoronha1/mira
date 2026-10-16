#!/usr/bin/env python3
"""Flag ROS topic and service names that do not follow the Mira topic schema.

Checks literal names at publisher/subscriber/service/client call sites, topic
settings in Python defaults and YAML, and ROS-side bridge names. Absolute
names must be ``/mira/<domain>/...`` (see utils/topics.py), a ROS-standard
interface, or a competition-contract topic. Relative names are allowed when
they resolve under a node's namespace (e.g. perception worker outputs), but not
as ``mira/...`` shorthand for an absolute name. Also checks
that utils/topics.py and utils/topics.hpp define the same names.

Usage: check_topics.py [PATH ...]   (default: this repository and ../mira_sim)
"""
import ast
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
SIM = ROOT.parent / 'mira_sim'
UTILS = ROOT / 'src/dependencies/utils'

DOMAINS = ('sensors', 'perception', 'odometry', 'state', 'guidance', 'control', 'safety', 'sim', 'replay')
SCHEMA = re.compile(r'^/mira/(%s)(/[a-z0-9_]+)+$' % '|'.join(DOMAINS))
STANDARD = {'/clock', '/tf', '/tf_static', '/diagnostics', '/joy', '/rosout', '/parameter_events'}
# External contracts kept on purpose (dnt_simulator competition bridges, SAUVC sim).
COMPETITION = {
    '/front_camera/left/image_raw', '/front_camera/right/image_raw', '/sim/front_camera/points',
    '/bottom_camera', '/side_cam', '/cmd_vel', '/world/pool_world/create',
}
SKIP_DIRS = {'build', 'install', 'log', '.venv', '.git', '__pycache__', 'ardupilot_gazebo', 'sauvc_sim',
             'bluerov2_gz', 'recordings', 'var', 'test', 'tests'}

CALL = re.compile(r'(?:create_publisher|create_subscription|create_service|create_client|Subscriber)'
                  r'\s*(?:<[^;()]*?>)?\s*\(\s*(?:[\w.:]+\s*,\s*)*?[\'"]([^\'"]+)[\'"]', re.S)
SETTING = re.compile(r'[\'"]\w*topic[\'"]\s*:\s*f?[\'"]([^\'"{}]+)[\'"]')
SOURCE = re.compile(r'ros2://(/[^\'"\s]*)')
YAML_KEY = re.compile(r'^\s*-?\s*(\w*topic|topic_name|ros_topic_name)\s*:\s*[\'"]?([^\'"\s#]+)', re.M)


def valid(name):
    if not name.startswith('/'):
        # Relative names resolve under the node namespace; a relative mira/... name
        # only works in the root namespace, so it must be written absolute.
        return not name.startswith(('mira/', 'vision/'))
    return name in STANDARD or name in COMPETITION or bool(SCHEMA.match(name))


def names(path):
    """Yield (line, name) for every checked literal in a source file."""
    text = path.read_text(errors='replace')
    patterns = [YAML_KEY] if path.suffix in ('.yaml', '.yml') else [CALL, SETTING]
    for pattern in [*patterns, SOURCE]:
        for match in pattern.finditer(text):
            name = match.group(match.lastindex)
            if '/' in name or pattern is not YAML_KEY:
                yield text.count('\n', 0, match.start(match.lastindex)) + 1, name


def files(roots):
    for root in roots:
        candidates = [root] if root.is_file() else root.rglob('*')
        for path in candidates:
            if path.suffix in ('.py', '.cpp', '.hpp', '.yaml', '.yml') and path.is_file() \
                    and not SKIP_DIRS & set(path.relative_to(root if root.is_dir() else root.parent).parts):
                yield path


def registry_mismatches():
    py = {}
    module = ast.parse((UTILS / 'utils/topics.py').read_text())
    sys.path.insert(0, str(UTILS))
    import utils.topics as topics
    for node in module.body:
        if isinstance(node, ast.Assign) and node.targets[0].id.isupper() and node.targets[0].id != 'ROOT':
            py[node.targets[0].id] = getattr(topics, node.targets[0].id)
    cpp = dict(re.findall(r'const char \*(\w+) = "([^"]+)"', (UTILS / 'include/utils/topics.hpp').read_text()))
    problems = [f'{key}: topics.py={py.get(key)!r} topics.hpp={cpp.get(key)!r}'
                for key in sorted(py.keys() | cpp.keys()) if py.get(key) != cpp.get(key)]
    problems += [f'{key}={value!r} is outside the schema' for key, value in py.items() if not valid(value)]
    return problems


def main(argv):
    roots = [Path(arg).resolve() for arg in argv] or [p for p in (ROOT, SIM) if p.exists()]
    problems = registry_mismatches()
    for path in files(roots):
        for line, name in names(path):
            if not valid(name):
                problems.append(f'{path}:{line}: {name}')
    for problem in problems:
        print(problem)
    print(f'{len(problems)} topic schema problem(s)' if problems else 'topic names follow the schema')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
