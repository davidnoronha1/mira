"""Unit conversions used at hardware protocol boundaries."""


STANDARD_GRAVITY = 9.80665


def millivolts_to_volts(value):
    return value / 1000.0


def hectopascals_to_pascals(value):
    return value * 100.0


def milligravity_to_meters_per_second_squared(value):
    return value * STANDARD_GRAVITY / 1000.0


def milliradians_per_second_to_radians_per_second(value):
    return value / 1000.0


def milligauss_to_tesla(value):
    return value * 1e-7
