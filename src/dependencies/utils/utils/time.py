"""Small, ROS-agnostic helpers for monotonic and ROS-clock time values."""

import time


NANOSECONDS_PER_SECOND = 1_000_000_000


def monotonic_now():
    """Return a monotonic timestamp in seconds for durations and deadlines."""
    return time.monotonic()


def seconds_from_rate(rate_hz):
    """Return the interval in seconds for a positive frequency in hertz."""
    if rate_hz <= 0:
        raise ValueError("Rate must be positive")
    return 1.0 / rate_hz


def clock_seconds(clock):
    """Return the current time from a ROS-like clock in seconds."""
    return clock.now().nanoseconds / NANOSECONDS_PER_SECOND


def stamp_seconds(stamp):
    """Return the seconds represented by a ROS-style sec/nanosec stamp."""
    return stamp.sec + stamp.nanosec / NANOSECONDS_PER_SECOND


def stamp_age(clock, stamp):
    """Return the signed age of a ROS-style timestamp using the given clock."""
    stamp_nanoseconds = stamp.sec * NANOSECONDS_PER_SECOND + stamp.nanosec
    return (clock.now().nanoseconds - stamp_nanoseconds) / NANOSECONDS_PER_SECOND


def is_fresh(age_seconds, timeout_seconds):
    """Return whether an age is nonnegative and no older than its timeout."""
    return 0.0 <= age_seconds <= timeout_seconds


def elapsed(since, now=None):
    """Return monotonic seconds elapsed since a prior monotonic timestamp."""
    return (monotonic_now() if now is None else now) - since
