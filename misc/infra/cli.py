import sys, os, argparse
from pathlib import Path
try:
    import argcomplete
    HAS_ARGCOMPLETE = True
except ImportError:
    HAS_ARGCOMPLETE = False

import misc.infra.state as state
from misc.infra.color import YELLOW, RESET, CYAN, BOLD
from misc.infra.shell import run_task_in_docker, find_matching_ros_targets
from misc.infra.tui import tui_select, _find_all_packages, _find_all_executables, _find_all_launch_files
from misc.infra.tasks import TASKS
from misc.infra.checks import check_ros
from misc.infra.targets import *
from misc.infra.config import WS_SOURCE


def _print_targets():
	print(f"\n{BOLD}Available targets:{RESET}\n")
	seen: set = set()
	for name, task_info in sorted(TASKS.items()):
		fn = task_info["fn"]
		if fn in seen:
			continue
		seen.add(fn)
		aliases = task_info.get("aliases", [])
		alias_str = f"  ({', '.join(aliases)})" if aliases else ""
		print(f"  {CYAN}{name:<26}{RESET}{alias_str:<14} {task_info['label']}")
	print()
	print(f"  {BOLD}Global flags:{RESET}  --dry-run   --docker   --list")
	print()


def _build_autocomplete_parser() -> argparse.ArgumentParser:
	"""Argparse parser used only for shell autocomplete (argcomplete hooks)."""
	parser = argparse.ArgumentParser(prog="mira.py", add_help=False)
	parser.add_argument("--dry-run", action="store_true")
	parser.add_argument("--docker", action="store_true")
	parser.add_argument("--list", "-l", action="store_true")

	sub = parser.add_subparsers(dest="command")

	# build / b — complete with package names
	for _cmd in ("build", "b"):
		sp = sub.add_parser(_cmd)
		a = sp.add_argument("package", nargs="?")
		if HAS_ARGCOMPLETE:
			a.completer = lambda prefix, **kw: _find_all_packages()

	# run / r — first arg: package or exe; second arg: exe in package
	def _run_completer(prefix, parsed, **kw):
		done = getattr(parsed, "args", []) or []
		if len(done) == 0:
			return _find_all_packages() + [e for _, e in _find_all_executables()]
		if len(done) == 1:
			return [e for p, e in _find_all_executables() if p == done[0]]
		return []

	for _cmd in ("run", "r"):
		sp = sub.add_parser(_cmd)
		a = sp.add_argument("args", nargs="*")
		if HAS_ARGCOMPLETE:
			a.completer = _run_completer

	# launch — first arg: package or launch file; second arg: launch file in package
	def _launch_completer(prefix, parsed, **kw):
		done = getattr(parsed, "args", []) or []
		if len(done) == 0:
			return _find_all_packages() + [f for _, f in _find_all_launch_files()]
		if len(done) == 1:
			return [f for p, f in _find_all_launch_files() if p == done[0]]
		return []

	sp = sub.add_parser("launch")
	a = sp.add_argument("args", nargs="*")
	if HAS_ARGCOMPLETE:
		a.completer = _launch_completer

	# help / h — complete with package names
	for _cmd in ("help", "h"):
		sp = sub.add_parser(_cmd)
		a = sp.add_argument("package", nargs="?")
		if HAS_ARGCOMPLETE:
			a.completer = lambda prefix, **kw: _find_all_packages()

	# service / svc — no static completions (services are discovered at runtime)
	for _cmd in ("service", "svc"):
		sub.add_parser(_cmd)

	# camera — complete with known camera names
	sp = sub.add_parser("camera")
	sp.add_argument("name", nargs="?", choices=CAMERA_OPTIONS)

	# alt-master — complete with /dev/ device paths
	sp = sub.add_parser("alt-master")
	a = sp.add_argument("port", nargs="?", default="/dev/Pixhawk")
	if HAS_ARGCOMPLETE:
		import argcomplete.completers as _ac
		a.completer = _ac.FilesCompleter(directories=False)

	# proxy-pixhawk, view-rtsp-stream, install-mavproxy — free-form args
	sub.add_parser("proxy-pixhawk").add_argument("laptop_ip", nargs="?")
	sub.add_parser("view-rtsp-stream").add_argument("rtsp_url", nargs="?")
	sub.add_parser("install-mavproxy").add_argument("python_version", nargs="?")

	# all remaining registered tasks (no specific arg completion needed)
	_handled = {
		"build", "b", "run", "r", "launch", "service", "svc",
		"camera", "alt-master", "proxy-pixhawk", "view-rtsp-stream", "install-mavproxy",
		"help", "h",
	}
	for _name in sorted(TASKS.keys()):
		if _name not in _handled:
			sub.add_parser(_name)

	return parser


def main():
	

	if HAS_ARGCOMPLETE:
		import argcomplete as _ac
		_ac.autocomplete(_build_autocomplete_parser())

	if not HAS_ARGCOMPLETE and "_ARGCOMPLETE" not in os.environ:
		print(f"  {YELLOW}Tip: enable tab-completion →  python mira.py enable-autocomplete{RESET}", file=sys.stderr)

	argv = sys.argv[1:]

	# ── Strip global flags anywhere in argv ─────────────────
	if "--dry-run" in argv:
		state.DRY_RUN = True
		argv = [a for a in argv if a != "--dry-run"]

	if "--docker" in argv:
		state.RUN_IN_DOCKER = True
		argv = [a for a in argv if a != "--docker"]

	# ── No command → build by default ───────────────────────
	if not argv:
		target_build()
		return

	if argv[0] in ("--list", "-l"):
		_print_targets()
		return

	if argv[0] in ("--help", "-h"):
		_print_targets()
		return

	cmd  = argv[0]
	rest = argv[1:]

	# ── Docker passthrough ───────────────────────────────────
	if state.RUN_IN_DOCKER:
		run_task_in_docker(sys.argv)
		return

	# ── Dispatch ─────────────────────────────────────────────

	# launch — positional args after the command
	if cmd == "launch":
		target_launch(*rest)
		return

	# run / r  — positional args after the command
	if cmd in ("run", "r"):
		target_run(*rest)
		return

	# service / svc
	if cmd in ("service", "svc"):
		target_service(*rest)
		return

	# build / b  — optional package as first positional or -p flag
	if cmd in ("build", "b"):
		pkg = None
		if rest:
			if rest[0] == "-p" and len(rest) > 1:
				pkg = rest[1]
			elif not rest[0].startswith("-"):
				pkg = rest[0]
		elif cmd == "b":
			packages = _find_all_packages()
			if packages:
				ALL = "(all packages)"
				choice = tui_select([ALL] + packages, title="Select Package to Build")
				if choice is None:
					return
				pkg = None if choice == ALL else choice
		target_build(pkg)
		return

	# camera  — optional name as positional arg
	if cmd == "camera":
		target_camera(rest[0] if rest else None)
		return

	# alt-master / alt_master  — optional port as positional arg
	if cmd in ("alt-master", "alt_master"):
		target_alt_master(rest[0] if rest else "/dev/Pixhawk")
		return

	# alt-master-sitl
	if cmd in ("alt-master-sitl", "alt_master_sitl"):
		target_alt_master_sitl()
		return

	# proxy-pixhawk  — optional laptop IP as positional arg
	if cmd in ("proxy-pixhawk", "proxy_pixhawk"):
		target_proxy_pixhawk(rest[0] if rest else None)
		return

	# install-mavproxy  — optional python version as positional arg
	if cmd in ("install-mavproxy", "install_mavproxy"):
		target_install_mavproxy(rest[0] if rest else "python3.12")
		return

	# view-rtsp-stream  — URL as positional arg
	if cmd in ("view-rtsp-stream", "view_rtsp_stream"):
		target_view_rtsp_stream(rest[0] if rest else None)
		return

	# help — optional package as positional arg
	if cmd in ("help", "h"):
		target_help(*rest)
		return

	# ── Task registry (no-arg tasks) ────────────────────────
	task_info = TASKS.get(cmd)
	if task_info is not None:
		task_info["fn"]()
		return

	# ── Fuzzy ROS target search ──────────────────────────────
	matches = find_matching_ros_targets(cmd)
	all_ros = (
		[("run",    p, n) for p, n in matches["executables"]] +
		[("launch", p, n) for p, n in matches["launch_files"]]
	)

	if not all_ros:
		error(f"Unknown target: '{cmd}'")
		print(f"  Run {CYAN}python mira.py --list{RESET} to see available targets.")
		print(f"  Run {CYAN}python mira.py run{RESET}    for a TUI node picker.")
		print(f"  Run {CYAN}python mira.py launch{RESET} for a TUI launch picker.")
		sys.exit(1)

	check_ros()

	if len(all_ros) == 1:
		kind, pkg, name = all_ros[0]
		header(f"{kind.title()}ing {pkg}/{name}...")
		run(f"{WS_SOURCE} && ros2 {kind} {pkg} {name}")
	else:
		item = tui_select(
			all_ros,
			title=f"Multiple matches for '{cmd}'",
			format_fn=lambda x: f"[{x[0]}] {x[1]}/{x[2]}",
		)
		if item:
			kind, pkg, name = item
			header(f"{kind.title()}ing {pkg}/{name}...")
			run(f"{WS_SOURCE} && ros2 {kind} {pkg} {name}")


if __name__ == "__main__":
	main()
