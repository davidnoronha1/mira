import socket
import subprocess
from PyQt6.QtCore import qInfo

def can_connect_to_device_ssh(ip: str, timeout: int = 5) -> bool:
	"""Check if an SSH service is reachable on the given IP address."""
	try:
		with socket.create_connection((ip, 22), timeout=timeout):
			qInfo(f"SSH connection to {ip}: SUCCESS")
			return True
	except (socket.timeout, ConnectionRefusedError, OSError) as e:
		qInfo(f"SSH connection to {ip}: FAILED ({e})")
		return False

def _lsusb_output() -> str:
	"""Helper to get lsusb output as string."""
	try:
		output = subprocess.check_output(["lsusb"], text=True)
		qInfo(f"lsusb output:\n{output}")
		return output
	except Exception as e:
		qInfo(f"lsusb failed: {e}")
		return ""

def is_pixhawk_connected() -> bool:
	"""Check if a Pixhawk (or similar flight controller) is connected via USB."""
	output = _lsusb_output().lower()
	pixhawk_keywords = ["3d robotics", "pixhawk", "holybro", "proficnc", "ardupilot", "mro"]
	found = any(kw in output for kw in pixhawk_keywords)
	qInfo(f"Pixhawk connected: {found}")
	return found

def is_camera_connected() -> bool:
	"""Check if a USB camera is connected via USB."""
	output = _lsusb_output().lower()
	camera_keywords = ["camera", "webcam", "uvc"]
	found = any(kw in output for kw in camera_keywords)
	qInfo(f"Camera connected: {found}")
	return found

def is_joystick_connected() -> bool:
	"""Check if a USB joystick/controller is connected via USB."""
	output = _lsusb_output().lower()
	joystick_keywords = ["joystick", "gamepad", "controller", "xbox", "playstation", "logitech"]
	found = any(kw in output for kw in joystick_keywords)
	qInfo(f"Joystick connected: {found}")
	return found