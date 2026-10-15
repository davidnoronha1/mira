import functools, os, subprocess, sys, shutil
from pathlib import Path
from typing import Optional
from misc.infra.color import info, msg, warn, error, header, step, GREEN, YELLOW, RESET, CYAN, BOLD
import misc.infra.state as state
from misc.infra.middleware import resolve_middleware


# DRY_RUN moved to state.py   # set via --dry-run flag; checked by run()
# RUN_IN_DOCKER moved to state.py  # set via --docker flag

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
	if state.DRY_RUN:
		print(f"   {YELLOW}[dry-run] skipping{RESET}")
		return subprocess.CompletedProcess(cmd, 0, stdout="", stderr="")
    
	env = {**_build_subprocess_env(), **(env_extra or {})}
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
	"""Return True if an NVIDIA GPU is usable on the host (driver responds)."""
	try:
		return subprocess.run(["nvidia-smi", "-L"], capture_output=True, timeout=5).returncode == 0
	except (OSError, subprocess.TimeoutExpired):
		return False


def has_docker_gpu() -> bool:
	"""Return True if Docker can start containers with the nvidia runtime."""
	try:
		result = subprocess.run(["docker", "info", "--format", "{{json .Runtimes}}"],
			capture_output=True, text=True, timeout=10)
	except (OSError, subprocess.TimeoutExpired):
		return False
	return result.returncode == 0 and '"nvidia"' in result.stdout


def _gpu_banner(ok: bool, title: str, detail: str) -> None:
	"""Print a boxed GPU/CPU verdict that is hard to miss in scrollback."""
	colour = "\033[1;42;30m" if ok else "\033[1;43;30m"
	bar = "#" * 104
	print(f"\n{colour}{bar}{RESET}", file=sys.stderr)
	for line in (title, "", detail):
		print(f"{colour}##  {line:<98}##{RESET}", file=sys.stderr)
	print(f"{colour}{bar}{RESET}\n", file=sys.stderr)


@functools.cache
def get_docker_service() -> str:
	"""Pick the GPU service only when it can start; MIRA_GPU=0/1 overrides."""
	forced = os.environ.get("MIRA_GPU")
	if forced == "1":
		_gpu_banner(True, "GPU FORCED (MIRA_GPU=1)", "Using the 'mira' service; it fails to start if the GPU is unusable.")
		return "mira"
	if forced == "0":
		_gpu_banner(False, "GPU DISABLED (MIRA_GPU=0) - RUNNING ON CPU", "Using the 'mira-nogpu' service.")
		return "mira-nogpu"
	if not has_cuda():
		_gpu_banner(False, "GPU NOT AVAILABLE - RUNNING ON CPU",
			"No working NVIDIA driver (nvidia-smi failed); using the 'mira-nogpu' service.")
		return "mira-nogpu"
	if not has_docker_gpu():
		_gpu_banner(False, "GPU NOT AVAILABLE TO DOCKER - RUNNING ON CPU",
			"Docker has no nvidia runtime: install nvidia-container-toolkit; using 'mira-nogpu'.")
		return "mira-nogpu"
	_gpu_banner(True, "GPU ENABLED", "NVIDIA GPU and Docker nvidia runtime found; using the 'mira' service.")
	return "mira"


def ensure_docker_container():
	"""Ensure the Docker container is running."""
	service = get_docker_service()
	step(f"Ensuring Docker container is running ({service})...")
	if not state.DRY_RUN:
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
	import shlex
	cmd_args = shlex.join(filtered_args)
	middleware = resolve_middleware()
	# Pass values at exec time: a persistent container may have been started
	# with a different profile. Paths under the workspace are translated.
	workspace = Path(__file__).resolve().parents[2]
	def container_value(value):
		return value.replace(str(workspace), "/workspace")
	env_args = " ".join(
		f"-e {shlex.quote(key + '=' + container_value(value))}"
		for key, value in middleware.items()
	)
	
	header(f"Running task in Docker container: {cmd_args}")
	
	docker_cmd = (
		f'docker compose exec {env_args} {service} bash -c '
		f'{shlex.quote("cd /workspace && python3 mira.py " + cmd_args)}'
	)
	
	run(docker_cmd)


def find_matching_ros_targets(name: str) -> dict:
	"""
	Search for ROS2 executables and launch files matching the given name.
	Supports exact, stem, and substring matches.

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
					if (item.is_file()
							and os.access(item, os.X_OK)
							and item.suffix not in {'.so', '.a', '.py'}
							and not item.name.startswith('lib')
							and (item.name == name or name in item.name)):
						results["executables"].append((package_dir.name, item.name))

	# Search for launch files in src/
	src_path = Path("src")
	if src_path.exists():
		for pattern in ["**/*.launch", "**/*.launch.py", "**/*.launch.xml"]:
			for lf in src_path.glob(pattern):
				if lf.name == name or lf.stem == name or name in lf.name or name in lf.stem:
					# Walk up to find the package (directory containing package.xml)
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
						results["launch_files"].append((package, lf.name))

	return results
