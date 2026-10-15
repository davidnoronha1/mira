"""Decode depth once per camera and share metric, immutable samples."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class DepthFrame:
    image: object
    header: object
    info: object
    message: object


def decode_depth(msg):
    if msg.encoding not in ('32FC1', '16UC1') or msg.width <= 0 or msg.height <= 0:
        raise ValueError('Depth must be a nonempty 32FC1 or 16UC1 image')
    dtype = np.dtype(('>' if msg.is_bigendian else '<') +
                     ('f4' if msg.encoding == '32FC1' else 'u2'))
    if msg.step < msg.width * dtype.itemsize or len(msg.data) < msg.height * msg.step:
        raise ValueError('Truncated depth image or invalid row stride')
    depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                       strides=(msg.step, dtype.itemsize)).astype(np.float32)
    if msg.encoding == '16UC1':
        depth *= 0.001
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    depth.flags.writeable = False
    return depth


def matches(depth, header, shape, max_skew):
    if depth is None or depth.image.shape != shape or depth.header.frame_id != header.frame_id:
        return False
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    return abs(stamp(depth.header) - stamp(header)) <= max_skew
