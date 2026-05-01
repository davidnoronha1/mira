import os, sys, subprocess, shutil, re
from pathlib import Path
from typing import Optional
import misc.infra.state as state
from misc.infra.color import info, msg, warn, error, header, step, BOLD, CYAN, RESET, YELLOW, GREEN
from misc.infra.shell import run, sh, exists, get_docker_service, ensure_docker_container
from misc.infra.tui import tui_select, _find_all_ros_targets, _find_all_launch_files, _find_all_executables, _find_all_packages, _ros_tui_fmt
from misc.infra.tasks import task
from misc.infra.config import ROS_SOURCE, COLCON_ARGS, WS_SOURCE, GSTREAMER_FIX, NPROC, MACHINE_IP, CMAKE_ARGS
from misc.infra.checks import check_ros, check_uv, validate_packages


def _print_build_issues() -> None:
	"""Print misc/doc/BUILD_ISSUES.md to the terminal with basic formatting."""
	guide = Path(__file__).resolve().parent / "misc" / "doc" / "BUILD_ISSUES.md"
	if not guide.exists():
		return
	print(f"\n{BOLD}{CYAN}{'─' * 60}{RESET}")
	print(f"{BOLD}{CYAN}  Build Troubleshooting Guide{RESET}")
	print(f"{BOLD}{CYAN}{'─' * 60}{RESET}\n")
	for line in guide.read_text().splitlines():
		if line.startswith("# "):
			pass  # already printed as header above
		elif line.startswith("## "):
			print(f"{BOLD}{YELLOW}  {line[3:]}{RESET}")
		elif line.startswith("```"):
			pass  # skip fence markers
		elif line.startswith("**Fix:**"):
			print(f"  {GREEN}→ Fix:{RESET}")
		elif line.strip().startswith("python mira.py"):
			print(f"      {CYAN}{line.strip()}{RESET}")
		elif line.startswith("---"):
			print()
		else:
			if line.strip():
				print(f"  {line}")
	print()


@task("Build the ROS workspace (or a single package with -p)", aliases=["b"])
def target_build(packages_select: Optional[str] = None):
	"""Build the ROS workspace (or a single package with -p)."""
	check_ros()
	header("Building workspace...")

	if packages_select:
		cmd = (
			f"{ROS_SOURCE} && source .venv/bin/activate && "
			f"colcon build {COLCON_ARGS} --packages-select {packages_select}"
		)
	else:
		cmd = f"{ROS_SOURCE} && source .venv/bin/activate && colcon build {COLCON_ARGS}"

	try:
		run(cmd)
	except subprocess.CalledProcessError:
		error("Build failed.")
		_print_build_issues()
		sys.exit(1)
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
			if not state.DRY_RUN:
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


@task("Forward Pixhawk telemetry via mavproxy to a laptop IP")
def target_proxy_pixhawk(laptop_ip: str = ""):
	"""Forward Pixhawk telemetry via mavproxy to a laptop IP."""
	if not laptop_ip:
		try:
			laptop_ip = input(f"  {CYAN}Laptop IP{RESET} (e.g. 192.168.2.XX): ").strip()
		except (EOFError, KeyboardInterrupt):
			print()
			return

	if not exists("mavproxy.py") and not exists("mavproxy"):
		error("mavproxy not found. Run: python mira.py install-mavproxy")
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
	prompt_py  = Path(__file__).resolve().parent / "prompt.py"

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


@task("Launch a ROS2 launch file or node — TUI picker if no args given")
def target_launch(*args):
	"""ros2 launch (or run) with combined TUI selection when no file is specified.

	Usage:
	    mira.py launch                        # combined TUI: launch files + executables by package
	    mira.py l <file>                      # search all packages for <file>
	    mira.py launch <package> <file>       # explicit package + file
	"""
	check_ros()

	if len(args) == 0:
		# Combined TUI: launch files and executables grouped by package
		all_targets = _find_all_ros_targets()
		if not all_targets:
			warn("No launch files or executables found. Have you built the workspace?")
			return
		item = tui_select(all_targets, title="Select Launch File / Executable", format_fn=_ros_tui_fmt)
		if item is None:
			return
		kind, package_name, name = item
		if kind == "exe":
			header(f"Running {package_name}/{name}...")
			run(f"{WS_SOURCE} && ros2 run {package_name} {name}")
		else:
			header(f"Launching {package_name}/{name}...")
			run(f"{WS_SOURCE} && ros2 launch {package_name} {name}")
		return

	launch_files = _find_all_launch_files()
	fmt = lambda x: f"{x[0]:<32} {x[1]}"

	if len(args) == 1:
		query = args[0]
		matches = [(p, f) for p, f in launch_files
		           if f == query or query in f or query in Path(f).stem]
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


@task("Run a ROS2 node or launch file — TUI picker if no args given", aliases=["r"])
def target_run(*args):
	"""ros2 run (or launch) with combined TUI selection when no node is specified.

	Usage:
	    mira.py run                           # combined TUI: launch files + executables by package
	    mira.py r <executable>               # search all packages for <executable>
	    mira.py run <package> <executable>   # explicit package + executable
	"""
	check_ros()

	if len(args) == 0:
		# Combined TUI: launch files and executables grouped by package
		all_targets = _find_all_ros_targets()
		if not all_targets:
			warn("No executables or launch files found. Build the workspace first.")
			return
		item = tui_select(all_targets, title="Select ROS2 Executable / Launch File", format_fn=_ros_tui_fmt)
		if item is None:
			return
		kind, package_name, name = item
		if kind == "exe":
			header(f"Running {package_name}/{name}...")
			run(f"{WS_SOURCE} && ros2 run {package_name} {name}")
		else:
			header(f"Launching {package_name}/{name}...")
			run(f"{WS_SOURCE} && ros2 launch {package_name} {name}")
		return

	executables = _find_all_executables()
	fmt = lambda x: f"{x[0]:<32} {x[1]}"

	if len(args) == 1:
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


@task("Open a root shell inside the mira Docker container", aliases=["docker"])
def target_shell_docker():
	"""Open a root shell inside the mira Docker container."""
	ensure_docker_container()
	service = get_docker_service()
	run("xhost +local:docker || true", hidden=True)
	run(f"docker compose exec -u root {service} /bin/bash")


@task("Ensure the Docker container is running (no-recreate)")
def target_docker_ensure():
	"""Ensure the Docker container is running."""
	ensure_docker_container()


@task("Fix workspace file ownership after Docker operations")
def target_docker_fix_perms():
	"""Restore workspace file ownership to the current user (undoes root writes from Docker)."""
	uid = os.getuid()
	gid = os.getgid()
	header("Fixing file permissions...")
	run(f"sudo chown -R {uid}:{gid} .")
	info("File ownership restored.")


def _print_markdown(text: str) -> None:
	"""Render basic markdown to the terminal."""
	in_code_block = False
	for line in text.splitlines():
		if line.startswith("```"):
			in_code_block = not in_code_block
			print(f"  {CYAN}{line}{RESET}" if in_code_block else f"  {CYAN}{line}{RESET}")
			continue
		if in_code_block:
			print(f"    {line}")
			continue
		if line.startswith("### "):
			print(f"\n  {BOLD}{line[4:]}{RESET}")
		elif line.startswith("## "):
			print(f"\n{BOLD}{YELLOW}  {line[3:]}{RESET}")
		elif line.startswith("# "):
			print(f"\n{BOLD}{CYAN}  {line[2:]}{RESET}")
		elif line.startswith("> "):
			print(f"  {YELLOW}│ {line[2:]}{RESET}")
		elif line.strip():
			rendered = re.sub(r'\*\*(.*?)\*\*', f'{BOLD}\\1{RESET}', line)
			rendered = re.sub(r'`(.*?)`', f'{CYAN}\\1{RESET}', rendered)
			print(f"  {rendered}")
		else:
			print()


def _render_mermaid(diagram: str) -> None:
	"""Render a Mermaid diagram using termaid, fall back to raw if unavailable."""
	try:
		import termaid
		print()
		print(termaid.render(diagram))
		print()
	except ImportError:
		warn("termaid not installed — showing raw diagram. Install: pip install termaid")
		print(f"\n{CYAN}```mermaid\n{diagram.strip()}\n```{RESET}\n")
	except Exception as e:
		warn(f"termaid rendering failed: {e}")
		print(f"\n{CYAN}```mermaid\n{diagram.strip()}\n```{RESET}\n")


def _render_readme(text: str, title: str) -> None:
	"""Render README.md to terminal, replacing mermaid blocks with rendered diagrams."""
	header(title)
	pattern = re.compile(r'```mermaid\n(.*?)```', re.DOTALL)
	last_end = 0
	for match in pattern.finditer(text):
		if match.start() > last_end:
			_print_markdown(text[last_end:match.start()])
		_render_mermaid(match.group(1))
		last_end = match.end()
	if last_end < len(text):
		_print_markdown(text[last_end:])


@task("Show README.md for a package — TUI picker if no arg given", aliases=["h"])
def target_help(*args):
	"""Display README.md for a ROS package in the terminal."""
	src_path = Path("src")

	if not args:
		packages = _find_all_packages()
		if not packages:
			warn("No packages found in src/")
			return
		pkg_name = tui_select(packages, title="Select Package")
		if pkg_name is None:
			return
	else:
		pkg_name = args[0]

	pkg_dir = None
	if src_path.exists():
		for pkg_xml in src_path.glob("**/package.xml"):
			if pkg_xml.parent.name == pkg_name:
				pkg_dir = pkg_xml.parent
				break

	if pkg_dir is None:
		error(f"Package '{pkg_name}' not found in src/")
		sys.exit(1)

	readme = pkg_dir / "README.md"
	if not readme.exists():
		warn(f"No README.md in {pkg_dir}")
		return

	_render_readme(readme.read_text(), pkg_name)


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


