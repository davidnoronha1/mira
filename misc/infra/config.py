import os
from misc.infra.shell import sh


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
	f"{SKIP_FLAGS}"
)

GSTREAMER_FIX = f"export LD_PRELOAD={sh('gcc -print-file-name=libunwind.so.8', hidden=True)}"
WS_SOURCE     = "source .venv/bin/activate && source install/setup.bash"
ROS_SOURCE    = "source /opt/ros/jazzy/setup.bash"


