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

	env = {**os.environ, **(env_extra or {})}
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
	uid = os.getuid()
	gid = os.getgid()
	
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
	
	run(docker_cmd, env_extra={"_UID": str(uid), "_GID": str(gid)})


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
NPROC  = str(os.cpu_count() or 4)

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
	f"{SKIP_FLAGS}"
)

GSTREAMER_FIX = f"export LD_PRELOAD={sh('gcc -print-file-name=libunwind.so.8', hidden=True)}"
WS_SOURCE     = "source .venv/bin/activate && source install/setup.bash"
ROS_SOURCE    = "source /opt/ros/jazzy/setup.bash"


# ─────────────────────────────────────────────
# Pre-flight checks  (reusable functions)
# ─────────────────────────────────────────────

def check_uv():
	if not exists("uv"):
		error("uv is not installed. Install with: curl -LsSf https://astral.sh/uv/install.sh | sh")
		sys.exit(1)
	if not Path(".venv").exists():
		warn("Python virtual environment not found at .venv — run: python dev.py install-deps")
	else:
		info("Virtual environment found at .venv.")

	for name, expected in [("python3", "/usr/bin/python3"), ("python3.12", "/usr/bin/python3.12")]:
		path = which_or_empty(name)
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
	header("Validating package.xml files...")
	run("find ./src -type f -name 'package.xml' -exec uv run ./util/package-utils/validate_package.py {} \\;")


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
	header("Opening workspace shell...")
	rcfile = (
		f"cd {Path('.').resolve()} && "
		f"source $HOME/.bashrc && "
		f"source {Path('.').resolve()}/.venv/bin/activate && "
		f"source {Path('.').resolve()}/install/setup.bash"
	)
	os.execlp("bash", "bash", "--rcfile", f"<(echo '{rcfile}')")


@task("Launch a camera node (--name bottomcam|frontcam|auto)")
def target_camera(name: str):
	"""Launch a camera node. name = bottomcam | frontcam | auto"""
	check_ros()
	if name == "auto":
		run(f"{WS_SOURCE} && {GSTREAMER_FIX} && ros2 launch mira2_perception camera_auto.launch")
	elif name == "bottomcam":
		run(
			f"{WS_SOURCE} && {GSTREAMER_FIX} && "
			f"ros2 launch mira2_perception camera_bottom.launch"
		)
	elif name == "frontcam":
		run(
			f"{WS_SOURCE} && {GSTREAMER_FIX} && "
			f"ros2 launch mira2_perception camera_front.launch"
		)
	else:
		error(f"Unknown camera name: {name}. Valid options: bottomcam, frontcam, auto")


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


@task("Find and run ROS2 launch files from workspace (--file to specify)")
def target_launch(launch_file: Optional[str] = None):
	"""Find and run ROS2 launch files from the workspace."""
	check_ros()
	
	# Find all launch files in the src directory
	src_path = Path("src")
	if not src_path.exists():
		error("src/ directory not found. Are you in the workspace root?")
		sys.exit(1)
	
	# Find all .launch, .launch.py, and .launch.xml files
	launch_files = []
	for pattern in ["**/*.launch", "**/*.launch.py", "**/*.launch.xml"]:
		launch_files.extend(src_path.glob(pattern))
	
	if not launch_files:
		warn("No launch files found in src/")
		return
	
	# Organize by package
	launch_by_package = {}
	for lf in launch_files:
		# Find the actual ROS2 package by looking for package.xml
		# Start from the launch file's parent and walk up until we find package.xml
		package = None
		current_dir = lf.parent
		while current_dir != src_path and current_dir.parent != src_path.parent:
			if (current_dir / "package.xml").exists():
				package = current_dir.name
				break
			current_dir = current_dir.parent
		
		# Fallback: use the directory directly under src/ if no package.xml found
		if not package and len(lf.parts) >= 2:
			package = lf.parts[1]
		
		if package:
			if package not in launch_by_package:
				launch_by_package[package] = []
			launch_by_package[package].append(lf)
	
	# If no launch file specified, list all available
	if not launch_file:
		header("Available launch files:")
		print()
		for package in sorted(launch_by_package.keys()):
			print(f"{BOLD}{CYAN}{package}:{RESET}")
			for lf in sorted(launch_by_package[package]):
				# Find the package directory to show relative path properly
				package_dir = None
				current_dir = lf.parent
				while current_dir != src_path and current_dir.parent != src_path.parent:
					if (current_dir / "package.xml").exists():
						package_dir = current_dir
						break
					current_dir = current_dir.parent
				
				# Show relative path from package
				if package_dir:
					rel_path = lf.relative_to(package_dir)
				else:
					rel_path = lf.relative_to(src_path / package)
				print(f"  • {lf.name:<40} {YELLOW}({rel_path.parent}){RESET}")
			print()
		
		print(f"\n{BOLD}Usage:{RESET}")
		print(f"  python mira.py launch --file <package> <launch_file>")
		print(f"  python mira.py launch --file mira2_perception camera_auto.launch")
		print()
		return
	
	# Parse the launch file argument (format: "package launch_file" or just "launch_file")
	parts = launch_file.split()
	if len(parts) == 2:
		package_name, file_name = parts
	elif len(parts) == 1:
		# Try to find the file in any package
		file_name = parts[0]
		matching_files = [lf for lf in launch_files if lf.name == file_name]
		
		if not matching_files:
			error(f"Launch file '{file_name}' not found in workspace")
			sys.exit(1)
		elif len(matching_files) > 1:
			error(f"Multiple launch files named '{file_name}' found. Please specify package:")
			for lf in matching_files:
				# Find the package name by looking for package.xml
				package = None
				current_dir = lf.parent
				while current_dir != src_path and current_dir.parent != src_path.parent:
					if (current_dir / "package.xml").exists():
						package = current_dir.name
						break
					current_dir = current_dir.parent
				
				if not package and len(lf.parts) >= 2:
					package = lf.parts[1]
				
				print(f"  • {package} {file_name}")
			sys.exit(1)
		
		# Use the single match
		launch_path = matching_files[0]
		# Find the package name by looking for package.xml
		package_name = None
		current_dir = launch_path.parent
		while current_dir != src_path and current_dir.parent != src_path.parent:
			if (current_dir / "package.xml").exists():
				package_name = current_dir.name
				break
			current_dir = current_dir.parent
		
		# Fallback to directory under src/ if no package.xml found
		if not package_name and len(launch_path.parts) >= 2:
			package_name = launch_path.parts[1]
	else:
		error("Invalid --file format. Use: --file <package> <launch_file> or --file <launch_file>")
		sys.exit(1)
	
	# Find the exact launch file
	if len(parts) == 2:
		if package_name not in launch_by_package:
			error(f"Package '{package_name}' not found or has no launch files")
			sys.exit(1)
		
		matching = [lf for lf in launch_by_package[package_name] if lf.name == file_name]
		if not matching:
			error(f"Launch file '{file_name}' not found in package '{package_name}'")
			sys.exit(1)
		
		launch_path = matching[0]
	
	# Run the launch file
	header(f"Launching {package_name}/{file_name}...")
	run(f"{WS_SOURCE} && ros2 launch {package_name} {file_name}")


@task("Find and run ROS2 nodes from workspace (--node to specify)")
def target_run(node_spec: Optional[str] = None):
	"""Find and run ROS2 executable nodes from the workspace."""
	check_ros()
	
	# Check if install directory exists
	install_path = Path("install")
	if not install_path.exists():
		error("install/ directory not found. Build the workspace first with: python mira.py build")
		sys.exit(1)
	
	# Find all executables in install/*/lib/*/
	executables_by_package = {}
	
	for package_dir in install_path.iterdir():
		if not package_dir.is_dir() or package_dir.name in ["_local_setup_util_sh.py", "COLCON_IGNORE"]:
			continue
		
		lib_dir = package_dir / "lib" / package_dir.name
		if lib_dir.exists() and lib_dir.is_dir():
			executables = []
			for item in lib_dir.iterdir():
				# Check if it's an executable file (not a directory, not a .so/.a library)
				if (item.is_file() and 
					os.access(item, os.X_OK) and 
					not item.suffix in ['.so', '.a', '.py'] and
					not item.name.startswith('lib')):
					executables.append(item.name)
			
			if executables:
				executables_by_package[package_dir.name] = sorted(executables)
	
	if not executables_by_package:
		warn("No executable nodes found in install/")
		info("Make sure you've built the workspace with: python mira.py build")
		return
	
	# If no node specified, list all available
	if not node_spec:
		header("Available ROS2 nodes:")
		print()
		for package in sorted(executables_by_package.keys()):
			print(f"{BOLD}{CYAN}{package}:{RESET}")
			for exe in executables_by_package[package]:
				print(f"  • {exe}")
			print()
		
		print(f"\n{BOLD}Usage:{RESET}")
		print(f"  python mira.py run --node <package> <executable>")
		print(f"  python mira.py run --node mira2_control_master alt_master")
		print(f"  python mira.py run --node <executable>  # if name is unique")
		print()
		return
	
	# Parse the node specification (format: "package executable" or just "executable")
	parts = node_spec.split()
	if len(parts) == 2:
		package_name, executable_name = parts
	elif len(parts) == 1:
		# Try to find the executable in any package
		executable_name = parts[0]
		matching_packages = [pkg for pkg, exes in executables_by_package.items() if executable_name in exes]
		
		if not matching_packages:
			error(f"Executable '{executable_name}' not found in workspace")
			sys.exit(1)
		elif len(matching_packages) > 1:
			error(f"Multiple packages have an executable named '{executable_name}'. Please specify package:")
			for pkg in matching_packages:
				print(f"  • {pkg} {executable_name}")
			sys.exit(1)
		
		# Use the single match
		package_name = matching_packages[0]
	else:
		error("Invalid --node format. Use: --node <package> <executable> or --node <executable>")
		sys.exit(1)
	
	# Verify the package and executable exist
	if package_name not in executables_by_package:
		error(f"Package '{package_name}' not found or has no executables")
		sys.exit(1)
	
	if executable_name not in executables_by_package[package_name]:
		error(f"Executable '{executable_name}' not found in package '{package_name}'")
		print(f"Available executables in {package_name}:")
		for exe in executables_by_package[package_name]:
			print(f"  • {exe}")
		sys.exit(1)
	
	# Run the node
	header(f"Running {package_name}/{executable_name}...")
	run(f"{WS_SOURCE} && ros2 run {package_name} {executable_name}")


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

def main():
	global DRY_RUN, RUN_IN_DOCKER

	parser = argparse.ArgumentParser(
		description="MIRA workspace build tool",
		formatter_class=argparse.RawDescriptionHelpFormatter,
	)
	parser.add_argument("target", nargs="?", help="Target to run")
	parser.add_argument("--list",       action="store_true",  help="List all targets")
	parser.add_argument("--dry-run",    action="store_true",  help="Print commands without running them")
	parser.add_argument("--docker",     action="store_true",  help="Run this task inside the Docker container")
	parser.add_argument("-p", "--package", metavar="PKG",     help="Package name (for b / build --packages-select)")
	parser.add_argument("--laptop-ip",  metavar="IP",         help="Laptop IP for proxy-pixhawk")
	parser.add_argument("--pixhawk-port", default="/dev/Pixhawk", help="Pixhawk device path (default: /dev/Pixhawk, SITL: tcp:localhost:5760, proxy: tcp:localhost:14551)")
	parser.add_argument("--name",       default="bottomcam",  help="Camera name for camera target")
	parser.add_argument("--python-version", default="python3.12", help="Python version for mavproxy install")
	parser.add_argument("--file",       metavar="LAUNCH",     help="Launch file for launch target (format: 'package file' or 'file')")
	parser.add_argument("--node",       metavar="NODE",       help="Node for run target (format: 'package executable' or 'executable')")
	args = parser.parse_args()

	DRY_RUN = args.dry_run
	RUN_IN_DOCKER = args.docker

	if args.list or not args.target:
		print(f"\n{BOLD}Available targets:{RESET}")
		# Only show unique tasks (not aliases) and sort them
		seen = set()
		for name, task_info in TASKS.items():
			fn = task_info["fn"]
			if fn not in seen:
				seen.add(fn)
				print(f"  {CYAN}{name:<22}{RESET} {task_info['label']}")
		print()
		return

	# If --docker flag is set, re-run this script inside Docker
	if RUN_IN_DOCKER:
		run_task_in_docker(sys.argv)
		return

	t = args.target.lower()

	# Look up the task in our registry
	task_info = TASKS.get(t)
	if task_info is None:
		# Target not found - search for matching ROS2 executables and launch files
		matches = find_matching_ros_targets(args.target)
		
		total_matches = len(matches["executables"]) + len(matches["launch_files"])
		
		if total_matches == 0:
			error(f"Unknown target: '{args.target}'")
			print(f"Run {CYAN}python mira.py --list{RESET} to see available targets.")
			print(f"Or run {CYAN}python mira.py run{RESET} to see ROS2 executables.")
			print(f"Or run {CYAN}python mira.py launch{RESET} to see launch files.")
			sys.exit(1)
		elif total_matches == 1:
			# Exactly one match - run it automatically
			if matches["executables"]:
				package, exe_name = matches["executables"][0]
				info(f"Found executable: {package}/{exe_name}")
				header(f"Running {package}/{exe_name}...")
				check_ros()
				run(f"{WS_SOURCE} && ros2 run {package} {exe_name}")
			else:
				package, launch_name = matches["launch_files"][0]
				info(f"Found launch file: {package}/{launch_name}")
				header(f"Launching {package}/{launch_name}...")
				check_ros()
				run(f"{WS_SOURCE} && ros2 launch {package} {launch_name}")
			return
		else:
			# Multiple matches - prefer executable over launch file
			if matches["executables"]:
				package, exe_name = matches["executables"][0]
				warn(f"Multiple matches found for '{args.target}', preferring executable")
				info(f"Running: {package}/{exe_name}")
				if matches["launch_files"]:
					print(f"  {YELLOW}Alternative: ros2 launch {matches['launch_files'][0][0]} {matches['launch_files'][0][1]}{RESET}")
				header(f"Running {package}/{exe_name}...")
				check_ros()
				run(f"{WS_SOURCE} && ros2 run {package} {exe_name}")
			else:
				# Only launch files matched (multiple of them)
				package, launch_name = matches["launch_files"][0]
				warn(f"Multiple launch files found for '{args.target}', using first match")
				info(f"Launching: {package}/{launch_name}")
				header(f"Launching {package}/{launch_name}...")
				check_ros()
				run(f"{WS_SOURCE} && ros2 launch {package} {launch_name}")
			return

	# Get the function and call it with appropriate arguments
	fn = task_info["fn"]
	fn_name = fn.__name__.replace("target_", "")
	
	# Map task names to their argument providers
	if fn_name in ["build", "b"]:
		fn(args.package)
	elif fn_name == "install_mavproxy":
		fn(args.python_version)
	elif fn_name == "proxy_pixhawk":
		fn(args.laptop_ip)
	elif fn_name == "camera":
		fn(args.name)
	elif fn_name == "alt_master":
		fn(args.pixhawk_port)
	elif fn_name == "launch":
		fn(args.file)
	elif fn_name == "run":
		fn(args.node)
	else:
		fn()


if __name__ == "__main__":
	main()