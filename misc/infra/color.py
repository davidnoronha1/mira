import sys
from pathlib import Path


BOLD  = "\033[1m"
RED   = "\033[31m"
GREEN = "\033[32m"
YELLOW= "\033[33m"
CYAN  = "\033[36m"
RESET = "\033[0m"
BLUE = "\033[34m"

def print_logo():
    logo_path = Path(__file__).resolve().parent / "dntlogo.txt"
    if logo_path.exists():
        print(f"{BOLD}{CYAN}{logo_path.read_text()}{RESET}", end="")

def info(msg: str):    print(f"{GREEN}✅ {msg}{RESET}")
def msg(msg: str):     print(f"{BLUE}ℹ️ {msg}{RESET}")
def warn(msg: str):    print(f"{YELLOW}⚠️  {msg}{RESET}")
def error(msg: str):   print(f"{RED}❌ {msg}{RESET}", file=sys.stderr)
def header(msg: str):  print(f"\n{BOLD}{CYAN}▶ {msg}{RESET}")
def step(msg: str):    print(f"   {CYAN}→{RESET} {msg}")


