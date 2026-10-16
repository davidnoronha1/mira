"""Shared optical-frame target geometry for aligned RGB/depth and SVO data."""
import numpy as np


def sample_bbox_points(point_cloud, x1, y1, x2, y2, steps=8):
    points = []
    width, height = point_cloud.get_width(), point_cloud.get_height()
    for x in np.linspace(max(0, x1), min(width - 1, x2), steps, dtype=int):
        for y in np.linspace(max(0, y1), min(height - 1, y2), steps, dtype=int):
            status, value = point_cloud.get_value(int(x), int(y))
            if int(status) == 0 and np.isfinite(value[:3]).all() and value[2] > 0:
                points.append(value[:3])
    return np.asarray(points, dtype=np.float32) if points else None


def fit_3d_bbox(points):
    points = np.asarray(points)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('No finite target points')
    low, high = np.percentile(points, [5, 95], axis=0)
    center, half = (low + high) / 2, (high - low) / 2
    signs = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                      [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]])
    return center, half, center + signs * half


def associate_depth(depth, bbox, matrix, min_depth=0.1, max_depth=20.0, min_points=6):
    """Median depth in the inner box; depth is meters, registered to RGB pixels."""
    cx, cy, width, height = bbox
    h, w = depth.shape
    x1, x2 = max(0, int(cx - width / 4)), min(w, int(cx + width / 4) + 1)
    y1, y2 = max(0, int(cy - height / 4)), min(h, int(cy + height / 4) + 1)
    samples = depth[y1:y2, x1:x2]
    valid = samples[np.isfinite(samples) & (samples >= min_depth) & (samples <= max_depth)]
    if valid.size < min_points:
        return None
    z = float(np.median(valid))
    fx, fy = matrix[0, 0], matrix[1, 1]
    if not np.isfinite(matrix).all() or fx <= 0 or fy <= 0:
        return None
    return ((cx - matrix[0, 2]) * z / fx, (cy - matrix[1, 2]) * z / fy, z)


def associate_frame_depth(depth, bbox, matrix, min_depth=0.1, max_depth=20.0, min_points=6):
    """Localize a hollow gate from its frame rather than background in the hole.

    Sample a narrow perimeter band. Use the foreground depth percentile, which
    tolerates background pixels around thin posts. This assumes a visible frame;
    it does not recover gate orientation or validate a traversable aperture.
    """
    cx, cy, width, height = bbox
    h, w = depth.shape
    x1, x2 = max(0, int(cx-width/2)), min(w, int(cx+width/2)+1)
    y1, y2 = max(0, int(cy-height/2)), min(h, int(cy+height/2)+1)
    if x2 <= x1 or y2 <= y1:
        return None
    crop = depth[y1:y2, x1:x2]
    yy, xx = np.indices(crop.shape)
    band = max(2, round(min(width, height)*.025))
    perimeter = (xx < band) | (xx >= crop.shape[1]-band) | (yy < band)
    valid = crop[perimeter & np.isfinite(crop) & (crop >= min_depth) & (crop <= max_depth)]
    if valid.size < min_points or not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        return None
    z = float(np.percentile(valid, 10))
    return ((cx-matrix[0, 2])*z/matrix[0, 0], (cy-matrix[1, 2])*z/matrix[1, 1], z)
