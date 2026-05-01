import os, sys
from pathlib import Path
from typing import Optional


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


def _find_all_packages() -> list[str]:
	"""Return sorted list of ROS package names from src/**/package.xml."""
	src_path = Path("src")
	if not src_path.exists():
		return []
	return sorted(
		p.parent.name
		for p in src_path.glob("**/package.xml")
		if not (p.parent / "COLCON_IGNORE").exists()
	)


def _find_all_ros_targets() -> list[tuple[str, str, str]]:
	"""Return [(kind, package, name), ...] sorted by package then type (launch before exe).

	kind is either 'launch' or 'exe'.
	"""
	combined = (
		[("launch", p, n) for p, n in _find_all_launch_files()] +
		[("exe",    p, n) for p, n in _find_all_executables()]
	)
	combined.sort(key=lambda x: (x[1], 0 if x[0] == "launch" else 1, x[2]))
	return combined


def _ros_tui_fmt(x: tuple[str, str, str]) -> str:
	"""Format a (kind, package, name) tuple for display in the TUI."""
	kind_label = "launch" if x[0] == "launch" else "exe   "
	return f"{x[1]:<32}  [{kind_label}]  {x[2]}"


