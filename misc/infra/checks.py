import os, sys, shutil
from pathlib import Path
from misc.infra.color import error, warn, info, step, header
from misc.infra.shell import run, exists


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
		warn("Python virtual environment not found at .venv — run: python mira.py install-deps")

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


def check_ros():
	check_uv()
	if not Path("/opt/ros/jazzy").exists():
		error("ROS Jazzy not found at /opt/ros/jazzy. Only ROS Jazzy is supported.")
		sys.exit(1)


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


