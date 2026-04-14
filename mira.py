#!/usr/bin/env python3
"""
mira.py — Python-based workspace build tool for MIRA.
Adds: colored output, shell shortcuts, command chaining, dry-run mode, Docker execution, and more.

Usage:
	python mira.py <target> [options]
	python mira.py --list              # List all targets
	python mira.py build --dry-run     # Preview commands without running them
	python mira.py build --docker      # Run build inside Docker container
	python mira.py clean --docker      # Run clean inside Docker container

Autocomplete:
	To enable shell autocomplete, run:
		source enable_autocomplete.sh
	Or manually:
		pip install argcomplete
		eval "$(register-python-argcomplete mira.py)"
"""

import argparse
import os
import platform
import shutil
import socket
import subprocess
import sys
from pathlib import Path
from typing import Optional

try:
	import argcomplete
	HAS_ARGCOMPLETE = True
except ImportError:
	HAS_ARGCOMPLETE = False

# ─────────────────────────────────────────────
# ANSI color helpers
# ─────────────────────────────────────────────

BOLD  = "\033[1m"
RED   = "\033[31m"
GREEN = "\033[32m"
YELLOW= "\033[33m"
CYAN  = "\033[36m"
RESET = "\033[0m"
BLUE = "\033[34m"

def info(msg: str):    print(f"{GREEN}✅ {msg}{RESET}")
def msg(msg: str):     print(f"{BLUE}ℹ️ {msg}{RESET}")
def warn(msg: str):    print(f"{YELLOW}⚠️  {msg}{RESET}")
def error(msg: str):   print(f"{RED}❌ {msg}{RESET}", file=sys.stderr)
def header(msg: str):  print(f"\n{BOLD}{CYAN}▶ {msg}{RESET}")
def step(msg: str):    print(f"   {CYAN}→{RESET} {msg}")


# ─────────────────────────────────────────────
# Shell helpers  ← the "shortcuts for calling shell commands"
# ─────────────────────────────────────────────

DRY_RUN = False   # set via --dry-run flag; checked by run()
RUN_IN_DOCKER = False  # set via --docker flag

def _build_subprocess_env() -> dict:
	"""Base environment for subprocesses: os.environ with the active venv stripped out.
	Commands that need the venv source it themselves (via WS_SOURCE / ROS_SOURCE)."""
	env = dict(os.environ)
	venv = env.pop("VIRTUAL_ENV", None)
	if venv:
		env["PATH"] = ":".join(
			p for p in env.get("PATH", "").split(":") if not p.startswith(venv + "/")
		)
		# Also clear the prompt decoration left by activate
		env.pop("PS1", None)
	env["_UID"] = str(os.getuid())
	env["_GID"] = str(os.getgid())
	return env

env_builtin = _build_subprocess_env()

def run(
	cmd: str,
	*,
	capture: bool = False,
	check: bool = True,
	env_extra: Optional[dict] = None,
	cwd: Optional[str] = None,
	hidden: bool = False
) -> subprocess.CompletedProcess:
	"""
	Run a shell command.

	Args:
		cmd:        Shell command string (passed to bash -c).
		capture:    If True, capture stdout/stderr and return them.
		check:      Raise CalledProcessError on non-zero exit.
		env_extra:  Extra environment variables to merge in.
		cwd:        Working directory override.

	Returns:
		subprocess.CompletedProcess — access .stdout / .returncode etc.

	Examples:
		run("make clean")
		out = run("git log -1 --oneline", capture=True).stdout
	"""
	if not hidden:
		step(cmd)
	if DRY_RUN:
		print(f"   {YELLOW}[dry-run] skipping{RESET}")
		return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")
    
	global env_builtin
	env = {**env_builtin, **(env_extra or {})}
	result = subprocess.run(
		cmd,
		shell=True,
		executable="/bin/bash",
		capture_output=capture,
		text=capture,
		check=False,          # we handle errors ourselves
		env=env,
		cwd=cwd,
	)
	if check and result.returncode != 0:
		error(f"Command failed (exit {result.returncode}): {cmd}")
		if capture and result.stderr:
			print(result.stderr, file=sys.stderr)
		raise subprocess.CalledProcessError(result.returncode, cmd, output=result.stdout, stderr=result.stderr)
	return result


def sh(cmd: str, **kwargs) -> str:
	"""
	Shortcut: run a command and return its stripped stdout as a string.

	Example:
		ip = sh("hostname -I | awk '{print $1}'")
		branch = sh("git rev-parse --abbrev-ref HEAD")
	"""
	return run(cmd, capture=True, **kwargs).stdout.strip()


def exists(cmd: str) -> bool:
	"""Return True if a command exists on PATH."""
	return bool(shutil.which(cmd))


def which_or_empty(cmd: str) -> str:
	return shutil.which(cmd) or ""


def has_cuda() -> bool:
	"""Check if CUDA is installed on the system."""
	# Check for nvidia-smi command
	if not shutil.which("nvidia-smi"):
		return False
	
	# Check for CUDA libraries
	cuda_paths = [
		"/usr/local/cuda",
		"/opt/cuda",
		Path.home() / ".cuda"
	]
	
	for cuda_path in cuda_paths:
		if Path(cuda_path).exists():
			return True
	
	# Try running nvidia-smi to verify GPU is accessible
	try:
		result = subprocess.run(
			["nvidia-smi"],
			capture_output=True,
			timeout=2,
			check=False
		)
		return result.returncode == 0
	except (subprocess.TimeoutExpired, FileNotFoundError):
		return False


def get_docker_service() -> str:
	"""Determine which Docker service to use based on CUDA availability."""
	if has_cuda():
		info("CUDA detected - using 'mira' service with GPU support")
		return "mira"
	else:
		info("No CUDA detected - using 'mira-nogpu' service")
		return "mira-nogpu"


def ensure_docker_container():
	"""Ensure the Docker container is running."""
	service = get_docker_service()
	step(f"Ensuring Docker container is running ({service})...")
	if not DRY_RUN:
		run(f"docker compose up --no-recreate -d {service}", hidden=True)


def run_task_in_docker(script_args: list[str]):
	"""
	Re-run this script inside the Docker container with the same arguments.
	
	Args:
		script_args: The sys.argv arguments to pass to the script inside Docker
	"""
	ensure_docker_container()
	
	# Enable X11 forwarding for GUI apps
	run("xhost +local:docker || true", hidden=True)
	
	# Get current user/group for proper file permissions (using Python instead of shell)
	# uid = os.getuid()
	# gid = os.getgid()
	
	# Determine which service to use
	service = get_docker_service()
	
	# Build the command to run inside Docker
	# Remove --docker flag from args to avoid infinite recursion
	filtered_args = [arg for arg in script_args[1:] if arg != "--docker"]
	cmd_args = " ".join(filtered_args)
	
	header(f"Running task in Docker container: {cmd_args}")
	
	docker_cmd = (
		f'docker compose exec {service} bash -c '
		f'"cd /workspace && python3 mira.py {cmd_args}"'
	)
	
	run(docker_cmd)


def find_matching_ros_targets(name: str) -> dict:
	"""
	Search for ROS2 executables and launch files matching the given name.
	
	Returns:
		dict with 'executables' and 'launch_files' keys, each containing list of tuples (package, name)
	"""
	results = {"executables": [], "launch_files": []}
	
	# Search for executables in install/
	install_path = Path("install")
	if install_path.exists():
		for package_dir in install_path.iterdir():
			if not package_dir.is_dir() or package_dir.name in ["_local_setup_util_sh.py", "COLCON_IGNORE"]:
				continue
			
			lib_dir = package_dir / "lib" / package_dir.name
			if lib_dir.exists() and lib_dir.is_dir():
				for item in lib_dir.iterdir():
					if (item.is_file() and 
						os.access(item, os.X_OK) and 
						not item.suffix in ['.so', '.a', '.py'] and
						not item.name.startswith('lib') and
						item.name == name):
						results["executables"].append((package_dir.name, item.name))
	
	# Search for launch files in src/
	src_path = Path("src")
	if src_path.exists():
		for pattern in ["**/*.launch", "**/*.launch.py", "**/*.launch.xml"]:
			for lf in src_path.glob(pattern):
				if lf.name == name or lf.stem == name:
					# Get package name (directory under src/)
					if len(lf.parts) >= 2:
						package = lf.parts[1]
						results["launch_files"].append((package, lf.name))
	
	return results


# ─────────────────────────────────────────────
# TUI helpers
# ─────────────────────────────────────────────

def tui_select(items: list, title: str = "Select", format_fn=None) -> Optional[any]:
	"""
	Interactive arrow-key + type-to-filter picker.
	Returns the selected item, or None if cancelled (Esc / q / Ctrl-C).
	Falls back to None silently if not running in a terminal.
	"""
	if not items:
		return None
	if not sys.stdout.isatty():
		return None
	if format_fn is None:
		format_fn = str

	import curses

	chosen = [None]

	def _run(stdscr):
		try:
			curses.curs_set(0)
		except curses.error:
			pass
		curses.use_default_colors()
		try:
			curses.init_pair(1, curses.COLOR_CYAN,  -1)               # title bar
			curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_CYAN) # selected row
			curses.init_pair(3, curses.COLOR_YELLOW, -1)               # filter line
		except curses.error:
			pass

		query  = ""
		cursor = 0
		offset = 0

		while True:
			stdscr.erase()
			h, w = stdscr.getmaxyx()

			filtered = [it for it in items
			            if not query or query.lower() in format_fn(it).lower()]

			cursor = min(cursor, max(0, len(filtered) - 1))

			# ── title
			try:
				stdscr.addstr(0, 0, f" {title} "[:w],
				              curses.color_pair(1) | curses.A_BOLD)
			except curses.error:
				pass

			# ── filter line
			try:
				stdscr.addstr(1, 0, f" /{query}"[:w], curses.color_pair(3))
			except curses.error:
				pass

			# ── separator
			try:
				stdscr.addstr(2, 0, ("─" * w)[:w])
			except curses.error:
				pass

			# ── list
			list_h = max(1, h - 5)
			if cursor < offset:
				offset = cursor
			elif cursor >= offset + list_h:
				offset = cursor - list_h + 1

			for i in range(list_h):
				idx = i + offset
				if idx >= len(filtered):
					break
				label = " " + format_fn(filtered[idx])
				try:
					if idx == cursor:
						stdscr.addstr(3 + i, 0, ("▶" + label)[:w],
						              curses.color_pair(2) | curses.A_BOLD)
					else:
						stdscr.addstr(3 + i, 0, (" " + label)[:w])
				except curses.error:
					pass

			# ── bottom hint
			hint = " ↑↓ navigate  Enter select  Esc/q cancel  type to filter"
			try:
				stdscr.addstr(h - 2, 0, ("─" * w)[:w])
				stdscr.addstr(h - 1, 0, hint[:w])
			except curses.error:
				pass

			stdscr.refresh()

			try:
				key = stdscr.get_wch()
			except curses.error:
				continue

			if key == curses.KEY_UP:
				cursor = max(0, cursor - 1)
			elif key == curses.KEY_DOWN:
				cursor = min(len(filtered) - 1, cursor + 1)
			elif key in ("\n", "\r", curses.KEY_ENTER):
				if filtered:
					chosen[0] = filtered[cursor]
				break
			elif key == "\x1b":          # Esc
				break
			elif key in ("\x7f", curses.KEY_BACKSPACE, "\x08"):
				query  = query[:-1]
				cursor = 0
				offset = 0
			elif key == "q" and not query:
				break
			elif isinstance(key, str) and key.isprintable():
				query += key
				cursor = 0
				offset = 0

	try:
		curses.wrapper(_run)
	except KeyboardInterrupt:
		pass

	return chosen[0]


def _find_all_launch_files() -> list[tuple[str, str]]:
	"""Return [(package, filename), ...] for every launch file under src/."""
	src_path = Path("src")
	if not src_path.exists():
		return []

	result: list[tuple[str, str]] = []
	seen: set[tuple[str, str]] = set()

	for pattern in ["**/*.launch", "**/*.launch.py", "**/*.launch.xml"]:
		for lf in sorted(src_path.glob(pattern)):
			package = None
			cur = lf.parent
			while cur != src_path and cur != cur.parent:
				if (cur / "package.xml").exists():
					package = cur.name
					break
				cur = cur.parent
			if not package and len(lf.parts) >= 2:
				package = lf.parts[1]
			if package:
				key = (package, lf.name)
				if key not in seen:
					seen.add(key)
					result.append(key)
	return result


def _find_all_executables() -> list[tuple[str, str]]:
	"""Return [(package, executable), ...] from install/*/lib/*/."""
	install_path = Path("install")
	if not install_path.exists():
		return []

	result: list[tuple[str, str]] = []
	for pkg_dir in sorted(install_path.iterdir()):
		if not pkg_dir.is_dir() or pkg_dir.name in {"_local_setup_util_sh.py", "COLCON_IGNORE"}:
			continue
		lib_dir = pkg_dir / "lib" / pkg_dir.name
		if lib_dir.exists():
			for item in sorted(lib_dir.iterdir()):
				if (item.is_file()
						and os.access(item, os.X_OK)
						and item.suffix not in {".so", ".a", ".py"}
						and not item.name.startswith("lib")):
					result.append((pkg_dir.name, item.name))
	return result


# ─────────────────────────────────────────────
# Task registration decorator
# ─────────────────────────────────────────────

TASKS = {}  # name -> {"fn": callable, "label": str, "aliases": list}

def task(label: str, aliases: Optional[list[str]] = None):
	"""
	Decorator to register a task function with its help text label.
	
	Args:
		label: Help text describing what the task does
		aliases: Optional list of alternative names for this task
	
	Example:
		@task("Build the ROS workspace", aliases=["b"])
		def build(packages_select: Optional[str] = None):
			...
	"""
	def decorator(fn):
		# Extract the task name from the function name (remove 'target_' prefix if present)
		# and convert underscores to hyphens for consistency with CLI conventions
		name = fn.__name__.replace("target_", "").replace("_", "-")
		TASKS[name] = {"fn": fn, "label": label, "aliases": aliases or []}
		# Also register aliases
		for alias in (aliases or []):
			TASKS[alias] = {"fn": fn, "label": label, "aliases": []}
		return fn
	return decorator


# ─────────────────────────────────────────────
# Environment / config
# ─────────────────────────────────────────────

os.environ["FORCE_COLOR"]               = "1"
os.environ["RCUTILS_COLORIZED_OUTPUT"]  = "1"
os.environ["RCUTILS_CONSOLE_OUTPUT_FORMAT"] = "{severity} {message}"
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
	"fifo_size;500000|overrun_nonfatal;1|fflags;nobuffer|"
	"flags;low_delay|framedrop;1|vf;setpts=0"
)

# Get machine IP using Python instead of shell command
def get_machine_ip() -> str:
	"""Get the machine's primary IP address."""
	try:
		# Create a socket to determine the primary IP
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.connect(("8.8.8.8", 80))
		ip = s.getsockname()[0]
		s.close()
		return ip
	except Exception:
		return "127.0.0.1"

MACHINE_IP   = get_machine_ip()
MACHINE_NAME = {"192.168.2.6": "ORIN", "192.168.2.4": "RPI4"}.get(MACHINE_IP, "UNKNOWN")
os.environ["MACHINE_IP"]   = MACHINE_IP
os.environ["MACHINE_NAME"] = MACHINE_NAME

LINKER = "lld"
# Get number of CPUs using Python instead of shell command
NPROC  = str((os.cpu_count() or 4) -1)

CMAKE_ARGS = " ".join([
	"-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
	"-DCMAKE_COLOR_DIAGNOSTICS=ON",
	"-GNinja",
	f"-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld={LINKER}",
	f"-DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld={LINKER}",
	f"-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld={LINKER}",
	"--no-warn-unused-cli",
])

SKIP_PACKAGES   = os.environ.get("SKIP_PACKAGES", "")
SKIP_FLAGS      = f"--packages-skip {SKIP_PACKAGES}" if SKIP_PACKAGES else ""

COLCON_ARGS = (
	f"--cmake-args {CMAKE_ARGS} "
	f"--parallel-workers {NPROC} "
	f"--event-handlers console_cohesion+ "
	f"--continue-on-error "
	f"--symlink-install "
	f"{SKIP_FLAGS}"
)

GSTREAMER_FIX = f"export LD_PRELOAD={sh('gcc -print-file-name=libunwind.so.8', hidden=True)}"
WS_SOURCE     = "source .venv/bin/activate && source install/setup.bash"
ROS_SOURCE    = "source /opt/ros/jazzy/setup.bash"


# ─────────────────────────────────────────────
# Pre-flight checks  (reusable functions)
# ─────────────────────────────────────────────

def _path_without_venv() -> str:
	"""Return PATH with any active virtualenv bin directory removed."""
	venv = os.environ.get("VIRTUAL_ENV", "")
	parts = os.environ.get("PATH", "").split(":")
	if venv:
		parts = [p for p in parts if not p.startswith(venv + "/")]
	return ":".join(parts)


def check_uv():
	if not exists("uv"):
		error("uv is not installed. Install with: curl -LsSf https://astral.sh/uv/install.sh | sh")
		sys.exit(1)
	if not Path(".venv").exists():
		warn("Python virtual environment not found at .venv — run: python dev.py install-deps")
	else:
		info("Virtual environment found at .venv.")

	# When mira.py itself is run inside the venv (e.g. `python3 mira.py`) the
	# venv's bin/ sits at the front of PATH, making `which python3` point there
	# instead of /usr/bin.  Strip the venv from PATH for this check only.
	in_venv = bool(os.environ.get("VIRTUAL_ENV"))
	sys_path = _path_without_venv() if in_venv else os.environ.get("PATH", "")

	for name, expected in [("python3", "/usr/bin/python3"), ("python3.12", "/usr/bin/python3.12")]:
		path = shutil.which(name, path=sys_path) or ""
		if not path:
			error(f"{name} not found in PATH"); sys.exit(1)
		if path != expected:
			error(f"{name} resolves to {path}. Expected {expected} (not ~/.local/bin)"); sys.exit(1)
		info(f"{name} → {path}")


def check_ros():
	check_uv()
	if not Path("/opt/ros/jazzy").exists():
		error("ROS Jazzy not found at /opt/ros/jazzy. Only ROS Jazzy is supported.")
		sys.exit(1)
	info("ROS Jazzy found.")


def validate_packages():
	"""Validate all ROS2 packages in the workspace using validate_package.py."""
	header("Validating packages...")
	
	# Find all package.xml files in src/
	src_path = Path("src")
	if not src_path.exists():
		warn("src/ directory not found")
		return
	
	package_xmls = list(src_path.glob("**/package.xml"))
	
	if not package_xmls:
		warn("No packages found in src/")
		return
	
	info(f"Found {len(package_xmls)} package(s) to validate")
	print()
	
	validation_script = Path("misc/util/package-utils/validate_package.py")
	if not validation_script.exists():
		warn(f"Validation script not found at {validation_script}")
		warn("Skipping detailed validation")
		return
	
	has_errors = False
	for pkg_xml in sorted(package_xmls):
		package_dir = pkg_xml.parent
		package_name = package_dir.name
		
		# Skip COLCON_IGNORE directories
		if (package_dir / "COLCON_IGNORE").exists():
			continue
		
		step(f"Validating {package_name}...")
		result = run(
			f"python3 {validation_script} {package_dir}",
			capture=True,
			check=False,
			hidden=True
		)
		
		if result.returncode != 0 or "❌" in result.stdout:
			has_errors = True
			print(result.stdout)
			if result.stderr:
				print(result.stderr)
		else:
			# Show summary only for successful validation
			if "✅" in result.stdout:
				print(f"      ✅ {package_name} validated successfully")
	
	print()
	if has_errors:
		warn("Some packages have validation issues (continuing anyway)")
	else:
		info("All packages validated successfully")


# ─────────────────────────────────────────────
# Targets
# ─────────────────────────────────────────────

@task("Build the ROS workspace (or a single package with -p)", aliases=["b"])
def target_build(packages_select: Optional[str] = None):
	"""Build the ROS workspace (or a single package with -p)."""
	check_ros()
	
	warn("If you built in docker last — you'll need to clean and rebuild")
	warn("If build fails due to CMakeCacheList issues, run: python dev.py clean")
	header("Building workspace...")

	if packages_select:
		cmd = (
			f"{ROS_SOURCE} && source .venv/bin/activate && "
			f"colcon build {COLCON_ARGS} --packages-select {packages_select}"
		)
	else:
		cmd = f"{ROS_SOURCE} && source .venv/bin/activate && colcon build {COLCON_ARGS}"

	run(cmd)
	info("Build complete.")


@task("Remove build/, install/, log/ directories")
def target_clean():
	"""Remove build/, install/, log/ directories."""
	header("Cleaning build artifacts...")
	# Use Python instead of shell command
	for dir_name in ["build", "install", "log"]:
		dir_path = Path(dir_name)
		if dir_path.exists():
			step(f"Removing {dir_name}/")
			if not DRY_RUN:
				shutil.rmtree(dir_path)
	info("Clean complete.")


@task("Install system + Python + ROS dependencies")
def target_install_deps():
	"""Install system + Python + ROS dependencies."""
	check_ros()
	check_uv()
	header("Installing build dependencies...")
	run("sudo apt install -y lld ninja-build build-essential cmake ros-jazzy-rmw-cyclonedds-cpp")
	header("Installing Python dependencies...")
	if not Path(".venv").exists():
		run("uv venv --system-site-packages")
	run("uv sync")
	header("Installing ROS dependencies...")
	run(f"{ROS_SOURCE} && rosdep install --from-paths src --ignore-src -r -y")
	info("All dependencies installed.")


@task("Install and patch mavproxy via uv tool")
def target_install_mavproxy(python_version: str = "python3.12"):
	"""Install and patch mavproxy via uv tool."""
	check_uv()
	header("Installing mavproxy...")
	run("uv tool install mavproxy")
	patch = "./misc/patches/mavproxy_rline_fix.patch"
	target = (
		f"/home/{os.environ['USER']}/.local/share/uv/tools/mavproxy/"
		f"lib/{python_version}/site-packages/MAVProxy/modules/lib/rline.py"
	)
	run(f"patch {target} < {patch}")
	info("mavproxy installed and patched.")


@task("Install udev rules for MIRA devices")
def target_install_udev():
	"""Install udev rules for MIRA devices."""
	header("Installing udev rules...")
	run("sudo cp misc/udev/96-mira.rules /etc/udev/rules.d/")
	run("sudo udevadm control --reload-rules")
	run("sudo udevadm trigger")
	info("udev rules installed.")


@task("Patch .vscode/settings.json paths to match this machine")
def target_fix_vscode():
	"""Patch .vscode/settings.json paths to match this machine."""
	header("Fixing VSCode settings paths...")
	settings = Path(".vscode/settings.json")
	if not settings.exists():
		warn("settings.json not found in .vscode/"); return
	current = str(Path(".").resolve())
	content = settings.read_text()
	patched = content.replace("/home/david/mira", current)
	settings.write_text(patched)
	info(f"Updated paths in {settings}")


@task("Init and update all git submodules")
def target_get_submodules():
	"""Init and update all git submodules."""
	header("Updating git submodules...")
	run("git submodule update --init --recursive")


@task("Hard-reset the current branch to match remote origin")
def target_force_update():
	"""Hard-reset the current branch to match remote origin."""
	header("Fetching latest changes from remote...")
	branch = sh("git rev-parse --abbrev-ref HEAD")
	run("git fetch origin")
	run(f"git reset --hard origin/{branch}")


@task("Print last git commit")
def target_repoversion():
	"""Print last git commit."""
	out = sh("git log -1 --oneline")
	print(f"Last commit: {out}")


@task("Validate all package.xml files in src/")
def target_validate_all():
	"""Validate all package.xml files in src/."""
	validate_packages()


@task("Enable shell autocomplete for mira.py")
def target_enable_autocomplete():
	"""Enable shell autocomplete for mira.py using argcomplete."""
	header("Setting up autocomplete for mira.py...")
	
	# Check if argcomplete is installed
	result = run(
		"python3 -c 'import argcomplete'",
		capture=True,
		check=False,
		hidden=True
	)
	
	if result.returncode != 0:
		info("Installing argcomplete...")
		run("uv pip install argcomplete")
	else:
		info("argcomplete is already installed")
	
	# Generate the autocomplete command
	mira_path = Path("mira.py").resolve()
	autocomplete_cmd = f'eval "$(register-python-argcomplete {mira_path})"'
	
	print()
	info("Autocomplete setup complete!")
	print()
	print(f"{BOLD}To enable autocomplete in your current shell, run:{RESET}")
	print(f"  {CYAN}{autocomplete_cmd}{RESET}")
	print()
	print(f"{BOLD}To make it permanent, add this line to your ~/.bashrc:{RESET}")
	print(f"  {CYAN}{autocomplete_cmd}{RESET}")
	print()
	print(f"{BOLD}Quick permanent setup:{RESET}")
	print(f"  {CYAN}echo '{autocomplete_cmd}' >> ~/.bashrc{RESET}")
	print()


@task("Forward Pixhawk telemetry via mavproxy to a laptop IP (requires --laptop-ip)")
def target_proxy_pixhawk(laptop_ip: str):
	"""Forward Pixhawk telemetry via mavproxy to a laptop IP."""
	if not laptop_ip:
		warn("--laptop-ip is not given, not proxying to laptop. Example: python dev.py proxy-pixhawk --laptop-ip 192.168.2.XX")
	if not exists("mavproxy.py") and not exists("mavproxy"):
		error("mavproxy not found. Run: python dev.py install-mavproxy")
		sys.exit(1)
	try: 
		run(f"uv run mavproxy.py --master=/dev/Pixhawk --baudrate 57600 " + (f" --out udp:{laptop_ip}:14550 " if laptop_ip else  "") + f"--out udp:{MACHINE_IP}:14551")
	except subprocess.CalledProcessError as e:
		msg("If you see the `no module named future` error, please apply the patch in misc/patches/mavproxy_rline_fix.patch and try again. OR edit the file and comment out the import")
		error(f"mavproxy exited with code {e.returncode}")


@task("Open an interactive bash shell with workspace sourced")
def target_shell():
	"""Open an interactive bash shell with workspace sourced."""
	import tempfile

	ws         = Path(".").resolve()
	prompt_py  = Path(__file__).resolve().parent / "misc" / "prompt.py"

	# Write an rc file to a temp path; bash reads it on startup.
	# We use delete=False because os.execlp() replaces this process — we can't
	# clean up ourselves, but /tmp is ephemeral so that's fine.
	with tempfile.NamedTemporaryFile(mode="w", suffix=".sh", delete=False, prefix="mira_rc_") as f:
		f.write(f"""\
# mira shell rc — auto-generated by mira.py shell
cd {ws}
[ -f "$HOME/.bashrc" ] && source "$HOME/.bashrc"
source {ws}/.venv/bin/activate
source {ws}/install/setup.bash
export PS1='$(python3 {prompt_py})'
""")
		rc_path = f.name

	header("Opening workspace shell  (exit to return)")
	os.execlp("bash", "bash", "--rcfile", rc_path)


@task("View an RTSP stream using ffplay (requires --rtsp-url)")
def target_view_rtsp_stream(rtsp_url: Optional[str] = None):
	"""View an RTSP stream using ffplay with low-latency settings."""
	if not rtsp_url:
		error("--rtsp-url is required. Example: python mira.py view-rtsp-stream --rtsp-url rtsp://192.168.2.6:8554/image_rtsp")
		sys.exit(1)
	
	if not exists("ffplay"):
		error("ffplay not found. Install with: sudo apt install ffmpeg")
		sys.exit(1)
	
	header(f"Opening RTSP stream: {rtsp_url}")
	info("Press 'q' to quit the stream")
	
	# Run ffplay with low-latency settings
	run(f'ffplay -fflags nobuffer -flags low_delay -framedrop -vf "setpts=0" {rtsp_url}')


CAMERA_OPTIONS = ["bottomcam", "frontcam", "auto", "zed"]

@task("Launch a camera node (bottomcam | frontcam | auto | zed) — TUI if no arg")
def target_camera(name: Optional[str] = None):
	"""Launch a camera node.  name = bottomcam | frontcam | auto | zed"""
	check_ros()
	if not name:
		name = tui_select(CAMERA_OPTIONS, title="Select Camera")
		if name is None:
			return
	if name == "auto":
		run(f"{WS_SOURCE} && {GSTREAMER_FIX} && ros2 launch mira2_perception camera_auto.launch.py")
	elif name == "bottomcam":
		run(f"{WS_SOURCE} && {GSTREAMER_FIX} && ros2 launch mira2_perception camera_bottom.launch.py")
	elif name == "frontcam":
		run(f"{WS_SOURCE} && {GSTREAMER_FIX} && ros2 launch mira2_perception camera_front.launch.py")
	elif name == "zed":
		run(f"{WS_SOURCE} && {GSTREAMER_FIX} && ros2 launch mira2_perception camera_zed.launch")
	else:
		error(f"Unknown camera: '{name}'. Valid options: {', '.join(CAMERA_OPTIONS)}")


@task("Launch alternative master control")
def target_alt_master(pixhawk_port: str = "/dev/Pixhawk"):
	"""Launch alternative master control."""
	check_ros()
	run(f"{WS_SOURCE} && ros2 launch mira2_control_master alt_master.launch pixhawk_address:={pixhawk_port}")

@task("Launch teleoperation")
def target_teleop():
	"""Launch teleoperation."""
	check_ros()
	run(f"{WS_SOURCE} && ros2 launch mira2_rov teleop.launch")


@task("Launch a ROS2 launch file — TUI picker if no args given", aliases=["l"])
def target_launch(*args):
	"""ros2 launch with TUI selection when no file is specified.

	Usage:
	    mira.py launch                        # interactive TUI picker
	    mira.py l <file>                      # search all packages for <file>
	    mira.py launch <package> <file>       # explicit package + file
	"""
	check_ros()

	launch_files = _find_all_launch_files()
	if not launch_files:
		warn("No launch files found in src/. Have you built the workspace?")
		return

	fmt = lambda x: f"{x[0]:<32} {x[1]}"

	if len(args) == 0:
		# Interactive TUI
		item = tui_select(launch_files, title="Select Launch File", format_fn=fmt)
		if item is None:
			return
		package_name, file_name = item

	elif len(args) == 1:
		query = args[0]
		matches = [(p, f) for p, f in launch_files
		           if f == query or query in f]
		if not matches:
			error(f"No launch file matching '{query}' found in workspace")
			sys.exit(1)
		if len(matches) == 1:
			package_name, file_name = matches[0]
		else:
			item = tui_select(matches, title=f"Matches for '{query}'", format_fn=fmt)
			if item is None:
				return
			package_name, file_name = item

	elif len(args) == 2:
		package_name, file_name = args[0], args[1]
		# Accept partial filename match within the given package
		if (package_name, file_name) not in launch_files:
			candidates = [(p, f) for p, f in launch_files
			              if p == package_name and (f == file_name or f.startswith(file_name))]
			if not candidates:
				error(f"Launch file '{file_name}' not found in package '{package_name}'")
				sys.exit(1)
			package_name, file_name = candidates[0]

	else:
		error("Usage: mira.py launch [package] [file]")
		sys.exit(1)

	header(f"Launching {package_name}/{file_name}...")
	run(f"{WS_SOURCE} && ros2 launch {package_name} {file_name}")


@task("Run a ROS2 node — TUI picker if no args given", aliases=["r"])
def target_run(*args):
	"""ros2 run with TUI selection when no node is specified.

	Usage:
	    mira.py run                           # interactive TUI picker
	    mira.py r <executable>               # search all packages for <executable>
	    mira.py run <package> <executable>   # explicit package + executable
	"""
	check_ros()

	executables = _find_all_executables()
	if not executables:
		warn("No executables found in install/. Build the workspace first.")
		return

	fmt = lambda x: f"{x[0]:<32} {x[1]}"

	if len(args) == 0:
		item = tui_select(executables, title="Select ROS2 Executable", format_fn=fmt)
		if item is None:
			return
		package_name, exe_name = item

	elif len(args) == 1:
		query = args[0]
		matches = [(p, e) for p, e in executables if e == query or query in e]
		if not matches:
			error(f"No executable matching '{query}' found in workspace")
			sys.exit(1)
		if len(matches) == 1:
			package_name, exe_name = matches[0]
		else:
			item = tui_select(matches, title=f"Matches for '{query}'", format_fn=fmt)
			if item is None:
				return
			package_name, exe_name = item

	elif len(args) == 2:
		package_name, exe_name = args[0], args[1]

	else:
		error("Usage: mira.py run [package] [executable]")
		sys.exit(1)

	header(f"Running {package_name}/{exe_name}...")
	run(f"{WS_SOURCE} && ros2 run {package_name} {exe_name}")


@task("Call a running ROS2 service — TUI picker if no args given", aliases=["svc"])
def target_service(*args):
	"""Discover live ROS2 services and call them with TUI selection.

	Usage:
	    mira.py svc                               # TUI picker from live services
	    mira.py svc <service>                     # call named service (auto-fill payload)
	    mira.py svc <service> <yaml_payload>      # call with explicit YAML payload
	"""
	check_ros()

	# Discover live services — needs a running ROS daemon
	result = run(f"{WS_SOURCE} && ros2 service list", capture=True, check=False, hidden=True)
	if result.returncode != 0 or not result.stdout.strip():
		warn("No services found. Is a ROS2 system running?")
		return

	services = sorted(s.strip() for s in result.stdout.strip().splitlines() if s.strip())

	if len(args) == 0:
		service_name = tui_select(services, title="Select ROS2 Service")
		if service_name is None:
			return
	elif len(args) >= 1:
		service_name = args[0]
	else:
		error("Usage: mira.py svc [service_name] [yaml_payload]")
		sys.exit(1)

	# Get the service type
	type_result = run(
		f"{WS_SOURCE} && ros2 service type {service_name}",
		capture=True, check=False, hidden=True,
	)
	if type_result.returncode != 0:
		error(f"Could not determine type for service: {service_name}")
		sys.exit(1)
	service_type = type_result.stdout.strip()

	# Determine payload
	if len(args) >= 2:
		payload = args[1]
	else:
		# Auto-fill for common simple types
		SIMPLE = {
			"std_srvs/srv/Trigger": "{}",
			"std_srvs/srv/Empty":   "{}",
		}
		if service_type in SIMPLE:
			payload = SIMPLE[service_type]
			info(f"Type: {service_type}  →  payload: {payload}")
		elif service_type == "std_srvs/srv/SetBool":
			raw = input(f"  {CYAN}SetBool data{RESET} [true/false]: ").strip().lower()
			payload = f"{{data: {'true' if raw in ('t', 'true', '1', 'y', 'yes') else 'false'}}}"
		else:
			info(f"Service type: {service_type}")
			run(f"ros2 interface show {service_type}", check=False)
			payload = input(f"  {CYAN}Request payload{RESET} (YAML, e.g. {{}}): ").strip() or "{}"

	header(f"Calling {service_name}...")
	run(f"{WS_SOURCE} && ros2 service call {service_name} {service_type} '{payload}'")


@task("Launch alt_master connected to ArduPilot SITL on localhost:5760")
def target_alt_master_sitl():
	"""Launch alt_master in SITL mode (ArduPilot SITL assumed on same host, port 5760)."""
	check_ros()
	run(f"{WS_SOURCE} && ros2 run mira2_control_master alt_master "
	    f"--ros-args -p pixhawk_address:=tcp:127.0.0.1:5760")


@task("Open a root shell inside the mira Docker container")
def target_shell_docker():
	"""Open a root shell inside the mira Docker container."""
	ensure_docker_container()
	service = get_docker_service()
	run("xhost +local:docker || true", hidden=True)
	run(f"docker compose exec -u root {service} /bin/bash")


@task("Full first-time workspace setup")
def target_setup():
	"""Full first-time workspace setup."""
	check_ros()
	target_install_deps()
	target_get_submodules()
	target_build()
	target_install_udev()
	target_fix_vscode()
	info("🚀 Complete workspace setup finished!")


# ─────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────

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


def main():
	global DRY_RUN, RUN_IN_DOCKER

	argv = sys.argv[1:]

	# ── Strip global flags anywhere in argv ─────────────────
	if "--dry-run" in argv:
		DRY_RUN = True
		argv = [a for a in argv if a != "--dry-run"]

	if "--docker" in argv:
		RUN_IN_DOCKER = True
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
	if RUN_IN_DOCKER:
		run_task_in_docker(sys.argv)
		return

	# ── Dispatch ─────────────────────────────────────────────

	# launch / l  — positional args after the command
	if cmd in ("launch", "l"):
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
