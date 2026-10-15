"""Resolve deployment middleware without importing ROS or changing node code."""

import json
import os
from pathlib import Path
import shlex
import tomllib


PROFILE_FILE = Path(__file__).resolve().parents[2] / "config/middleware/profiles.toml"
CONFIG_VARIABLES = (
    "CYCLONEDDS_URI", "FASTDDS_DEFAULT_PROFILES_FILE", "FASTRTPS_DEFAULT_PROFILES_FILE",
    "ZENOH_SESSION_CONFIG_URI", "ZENOH_ROUTER_CONFIG_URI", "ZENOH_CONFIG_OVERRIDE",
    "ZENOH_ROUTER_CHECK_ATTEMPTS", "ROS_LOCALHOST_ONLY",
    "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_STATIC_PEERS",
)


def profiles():
    with PROFILE_FILE.open("rb") as stream:
        return tomllib.load(stream)


def resolve_middleware(selection=None, environ=None):
    """Explicit profile > MIRA_MIDDLEWARE > existing RMW > CycloneDDS.

    Native middleware configuration is preserved. Profiles do not guess remote
    endpoints or enable shared memory. Environment output contains shell-safe
    values; it can also be passed directly to subprocesses.
    """
    env = os.environ if environ is None else environ
    available = profiles()
    name = selection or env.get("MIRA_MIDDLEWARE")
    if not name:
        implementation = env.get("RMW_IMPLEMENTATION")
        name = next((key for key, value in available.items()
                     if value["implementation"] == implementation), None)
        if implementation and name is None:
            raise ValueError(f"Unsupported RMW_IMPLEMENTATION: {implementation}")
        name = name or "cyclonedds"
    if name not in available:
        raise ValueError(f"Unknown middleware {name!r}; choose {', '.join(available)}")
    profile = available[name]
    domain = env.get("ROS_DOMAIN_ID", "0")
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise ValueError("ROS_DOMAIN_ID must be an integer between 0 and 232")
    result = {key: env[key] for key in CONFIG_VARIABLES if env.get(key)}
    result.update(MIRA_MIDDLEWARE=name, RMW_IMPLEMENTATION=profile["implementation"],
                  ROS_DOMAIN_ID=domain)
    if env.get("MIRA_MIDDLEWARE") == "zenoh-client" and not profile.get("remote_client"):
        # Undo client-mode fields when explicitly switching back to an onboard
        # profile. An empty export clears the previously exported override.
        result["ZENOH_CONFIG_OVERRIDE"] = _without_client_fields(
            result.get("ZENOH_CONFIG_OVERRIDE", ""))
    if profile.get("remote_client"):
        endpoint = env.get("MIRA_ZENOH_ENDPOINT", "")
        if not endpoint or not endpoint.startswith(("tcp/", "tls/")):
            raise ValueError("zenoh-client requires MIRA_ZENOH_ENDPOINT=tcp/HOST:PORT (or tls/)")
        # Reject separators that could inject another Zenoh override.
        if any(char in endpoint for char in ";\n\r"):
            raise ValueError("Invalid MIRA_ZENOH_ENDPOINT")
        result["MIRA_ZENOH_ENDPOINT"] = endpoint
        # Resolving an already-exported client profile must be idempotent.
        overrides = _without_client_fields(result.get("ZENOH_CONFIG_OVERRIDE", ""))
        result["ZENOH_CONFIG_OVERRIDE"] = ";".join(filter(None, (
            overrides, 'mode="client"',
            "connect/endpoints=" + json.dumps([endpoint]),
        )))
    return result


def _without_client_fields(overrides):
    return ";".join(part for part in overrides.split(";")
                    if part.strip() and part.strip().split("=", 1)[0].strip()
                    not in {"mode", "connect/endpoints"})


def shell_exports(environment):
    return "\n".join(f"export {key}={shlex.quote(value)}"
                     for key, value in sorted(environment.items()))
