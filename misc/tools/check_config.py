#!/usr/bin/env python3
"""Fail a build/release before packaging malformed or unsafe configuration."""
import math
from pathlib import Path
import re
import sys
import tomllib
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/dependencies/utils'))
from utils.parameters import flatten


def ros_parameters(path):
    """Expand nested ROS parameter sections into their dotted parameter names."""
    data = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    return flatten(data)


def finite_values(value, path):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f'{path}: nonfinite value')
    if isinstance(value, dict):
        for key, item in value.items():
            finite_values(item, f'{path}.{key}')
            if isinstance(item, (int, float)) and not isinstance(item, bool):
                if ('effort' in key or re.fullmatch(r'stage_\d+_(forward|lateral|thrust|yaw|roll|pitch)', key)) and not -1 <= item <= 1:
                    raise ValueError(f'{path}.{key}: effort outside [-1, 1]')
    elif isinstance(value, list):
        for item in value:
            finite_values(item, path)


def main():
    for path in (ROOT / 'config').rglob('*.toml'):
        with path.open('rb') as stream:
            finite_values(tomllib.load(stream), str(path))
    directories = [ROOT / 'src' / name / 'config' for name in (
        'mira2_hardware', 'mira2_control', 'mira2_path_planning',
        'mira2_perception', 'mira2_vision')]
    for directory in directories:
        for path in directory.glob('*.yaml'):
            data = yaml.safe_load(path.read_text())
            if not isinstance(data, dict):
                raise ValueError(f'{path}: expected ROS parameter mapping')
            finite_values(data, str(path))
    master_config = ROOT / 'src/mira2_hardware/config'
    calibration = ros_parameters(master_config / 'command_calibration.yaml')
    for axis in ('forward', 'lateral', 'thrust', 'pitch', 'roll', 'yaw'):
        if not 0 < calibration[f'{axis}.minimum'] < calibration[f'{axis}.neutral'] < calibration[f'{axis}.maximum'] < 65535:
            raise ValueError(f'{axis}: invalid PWM calibration')
        if type(calibration[f'{axis}.inverted']) is not bool or not 0 <= calibration[f'{axis}.deadband'] < 1:
            raise ValueError(f'{axis}: invalid inversion/deadband')
    runtime = ros_parameters(master_config / 'master.yaml')
    if calibration.keys() & runtime.keys():
        raise ValueError('Runtime and command calibration parameters overlap')
    if type(runtime['baudrate']) is not int or runtime['baudrate'] <= 0:
        raise ValueError('baudrate must be a positive integer')
    if not isinstance(runtime['initial_mode'], str) or not runtime['initial_mode'].strip():
        raise ValueError('initial_mode must be a nonempty string')
    plugins = runtime['plugins']
    if not isinstance(plugins, list) or not all(isinstance(name, str) and name for name in plugins):
        raise ValueError('plugins must be a list of nonempty strings')
    if len(plugins) != len(set(plugins)):
        raise ValueError('Duplicate controller plugins')
    for path in (ROOT / 'src/mira2_actions/config').glob('*.xml'):
        for element in ET.parse(path).iter():
            for key, value in element.attrib.items():
                if 'pwm' in key:
                    raise ValueError(f'{path}: obsolete PWM port {key}')
                if 'effort' in key and not value.startswith('{') and not -1 <= float(value) <= 1:
                    raise ValueError(f'{path}: invalid effort {key}')
    print('Configuration accepted')


if __name__ == '__main__':
    main()
