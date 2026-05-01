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
import re
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

import sys
from pathlib import Path
# Ensure misc/infra is importable if run directly
sys.path.insert(0, str(Path(__file__).resolve().parent))

from misc.infra.cli import main

if __name__ == '__main__':
    main()
