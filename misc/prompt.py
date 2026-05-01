#!/usr/bin/env python3
"""
misc/prompt.py — PS1 prompt for the mira workspace shell.

Shows: git branch (dirty flag), Python venv, Docker status,
       workspace sourced state, and time since last build.

Used via:   export PS1='$(python3 /path/to/misc/prompt.py)'

ANSI codes are wrapped in \\x01...\\x02 so bash readline correctly
accounts for non-printing characters when calculating line length.
"""

import os
import subprocess
import time
from pathlib import Path

# ── ANSI escape sequences ─────────────────────────────────────────────────────

RESET   = "\033[0m"
BOLD    = "\033[1m"
RED     = "\033[31m"
GREEN   = "\033[32m"
YELLOW  = "\033[33m"
BLUE    = "\033[34m"
MAGENTA = "\033[35m"
CYAN    = "\033[36m"
GREY    = "\033[90m"

def _c(text: str, *ansi: str) -> str:
    """Wrap text in readline-safe ANSI codes (\\x01 = RL_START, \\x02 = RL_END)."""
    start = "".join(f"\x01{a}\x02" for a in ansi)
    end   = f"\x01{RESET}\x02"
    return f"{start}{text}{end}"


# ── Git ───────────────────────────────────────────────────────────────────────

def _git_info() -> tuple[str | None, bool]:
    """Return (branch_name, is_dirty), or (None, False) outside a repo."""
    try:
        branch = subprocess.check_output(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            stderr=subprocess.DEVNULL, timeout=1,
        ).decode().strip()

        status = subprocess.check_output(
            ["git", "status", "--porcelain"],
            stderr=subprocess.DEVNULL, timeout=1,
        ).decode().strip()

        return branch, bool(status)
    except Exception:
        return None, False


# ── Python venv ───────────────────────────────────────────────────────────────

def _venv_name() -> str | None:
    venv = os.environ.get("VIRTUAL_ENV", "")
    return Path(venv).name if venv else None


# ── Docker ────────────────────────────────────────────────────────────────────

def _in_docker() -> bool:
    if Path("/.dockerenv").exists():
        return True
    try:
        with open("/proc/1/cgroup") as f:
            return "docker" in f.read()
    except Exception:
        return False


# ── Workspace sourced ─────────────────────────────────────────────────────────

def _ws_sourced() -> bool:
    """True when install/setup.bash has been sourced (install/ appears in AMENT_PREFIX_PATH)."""
    ament = os.environ.get("AMENT_PREFIX_PATH", "")
    if not ament:
        return False
    install = str(Path("install").resolve())
    return any(p.startswith(install) for p in ament.split(":"))


# ── Time since last build ─────────────────────────────────────────────────────

def _build_age() -> str | None:
    """Human-readable age of the most recent build, or None if never built."""
    mtimes = []
    for d in (Path("install"), Path("build")):
        if d.exists():
            mtimes.append(d.stat().st_mtime)
    if not mtimes:
        return None

    elapsed = time.time() - max(mtimes)
    if elapsed < 60:
        return f"{int(elapsed)}s"
    elif elapsed < 3600:
        m, s = divmod(int(elapsed), 60)
        return f"{m}m{s:02d}s"
    else:
        h, rem = divmod(int(elapsed), 3600)
        m = rem // 60
        return f"{h}h{m:02d}m"


# ── Assemble ──────────────────────────────────────────────────────────────────

def main() -> None:
    segments: list[str] = []

    # git branch
    branch, dirty = _git_info()
    if branch is not None:
        label = f"{branch}{'*' if dirty else ''}"
        color = YELLOW if dirty else CYAN
        segments.append(_c(label, color))

    # python venv
    venv = _venv_name()
    if venv:
        segments.append(_c(f"py:{venv}", MAGENTA))

    # docker
    if _in_docker():
        segments.append(_c("docker", BLUE, BOLD))

    # workspace sourced
    if _ws_sourced():
        segments.append(_c("ws✓", GREEN))
    else:
        segments.append(_c("ws✗", RED))

    # build age
    age = _build_age()
    if age:
        segments.append(_c(f"built {age} ago", GREY))
    else:
        segments.append(_c("never built", RED))

    sep = _c(" · ", GREY)
    body = sep.join(segments)
    arrow = _c("❯", BOLD, GREEN)

    print(f"{body} {arrow} ", end="", flush=True)


if __name__ == "__main__":
    main()
