import cv2
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional

def _order_corners_clockwise(corners: np.ndarray) -> np.ndarray:
    """Order 4 corners clockwise starting from top-left."""
    # Find center
    center = np.mean(corners, axis=0)
    
    # Calculate angles from center
    angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
    
    # Sort by angle (clockwise)
    sorted_indices = np.argsort(angles)
    ordered = corners[sorted_indices]
    
    # Ensure we start from top-left (smallest x+y sum typically)
    sums = ordered[:, 0] + ordered[:, 1]
    min_idx = np.argmin(sums)
    
    # Rotate array to start from top-left
    return np.roll(ordered, -min_idx, axis=0)

def isRectangleFromApproximation(
    cnt: np.ndarray, 
    frame: Optional[np.ndarray] = None, 
    angle_tolerance: float = 12, 
    side_ratio_tolerance: float = 0.25,
    epsilon_factor: float = 0.02,
    merge_distance_factor: float = 0.05,
    viz: bool = False
) -> Tuple[bool, Optional[np.ndarray]]:
    """
    Determine if a contour is approximately a rectangle by approximating the contour
    and checking if the result has 4 corners forming a rectangular shape.
    
    Args:
        cnt: Input contour
        frame: Optional frame for visualization
        angle_tolerance: Maximum deviation from 90° for corners (degrees)
        side_ratio_tolerance: Maximum ratio difference for opposite sides
        epsilon_factor: Approximation accuracy as fraction of perimeter (0.01-0.05 typical)
        merge_distance_factor: Points closer than this fraction of perimeter are merged (default 0.05)
        viz: Whether to visualize the detection
    
    Returns:
        (is_rectangle, corners) tuple where corners are ordered clockwise
    """
    
    if len(cnt) < 4:
        return False, None
    
    # Approximate the contour to a polygon
    perimeter = cv2.arcLength(cnt, True)
    epsilon = epsilon_factor * perimeter
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    
    # Reshape to 2D array of points
    points = approx.reshape(-1, 2).astype(np.float32)
    
    # Merge nearby points
    merge_distance = merge_distance_factor * perimeter
    merged_points = _merge_nearby_points(points, merge_distance)
    
    # Check if we got exactly 4 points after merging
    if len(merged_points) != 4:
        if viz and frame is not None:
            _visualize(frame, merged_points, False, 
                      f"Not 4 points ({len(merged_points)} after merging from {len(points)})")
        return False, None
    
    corners = merged_points
    
    # Order corners clockwise from top-left
    corners = _order_corners_clockwise(corners)
    
    # Check if it forms a proper rectangle
    is_rect, angles, side_ratios = _check_rectangle_properties(
        corners, angle_tolerance, side_ratio_tolerance
    )
    
    # Visualization
    if viz and frame is not None:
        status = f"Rectangle (angles: {[f'{a:.1f}°' for a in angles]})"
        if not is_rect:
            status = f"Not Rectangle (angles: {[f'{a:.1f}°' for a in angles]})"
        _visualize(frame, corners, is_rect, status)
    
    return is_rect, corners

def visualize_convex_hull(frame, hull):
	"""Visualize convex hull with semi-transparent overlay and outline.
	
	Args:
		frame: Input frame to draw on
		hull: Convex hull points (Nx1x2 or Nx2 array)
		
	Returns:
		vis_frame: Frame with hull visualization
	"""

	if hull is None:
		return frame.copy()

	vis_frame = frame.copy()
	
	# Fill hull with semi-transparent overlay
	hull_pts = hull.reshape(-1, 2).astype(np.int32)
	overlay = vis_frame.copy()
	cv2.fillConvexPoly(overlay, hull_pts, (0, 255, 0))            # green fill
	vis_frame = cv2.addWeighted(overlay, 0.3, vis_frame, 0.7, 0)    # blend

	# Draw hull outline and hull points
	cv2.polylines(vis_frame, [hull_pts], isClosed=True, color=(0, 128, 0), thickness=2)
	for pt in hull_pts:
		cv2.circle(vis_frame, tuple(int(x) for x in pt), 4, (0, 0, 255), -1)
	
	return vis_frame

def compute_convex_hull(contours, angle_tolerance=15, epsilon_ratio=0.02, merge_distance_factor=0.05):
    """
    Compute convex hull from multiple contours and find the best 4 points forming a rectangle.

    Args:
        contours: List of contours
        angle_tolerance: Allowed deviation (in degrees) from 90° for rectangle corners
        epsilon_ratio: Ratio used in approxPolyDP for corner simplification
        merge_distance_factor: Distance threshold for merging nearby points (as fraction of perimeter)

    Returns:
        hull: Convex hull of the best 4 rectangle points (4x1x2 array), or None if not found
    """
    if len(contours) == 0:
        return None

    # 1. Concatenate all contour points into a single (N,2) array
    all_pts = np.vstack(contours).reshape(-1, 2)

    # 2. Compute initial convex hull
    hull = cv2.convexHull(all_pts)
    hull_pts = hull.reshape(-1, 2).astype(np.float32)

    # 3. Approximate the hull to simplify it
    perimeter = cv2.arcLength(hull, True)
    epsilon = epsilon_ratio * perimeter
    approx = cv2.approxPolyDP(hull, epsilon, True)
    approx_pts = approx.reshape(-1, 2).astype(np.float32)

    # 4. Merge nearby points in the approximation
    merge_distance = merge_distance_factor * perimeter
    merged_pts = _merge_nearby_points(approx_pts, merge_distance)

    # 5. Find the best 4 points that form the most perfect rectangle
    if len(merged_pts) < 4:
        return None
    
    best_rect_points = _find_best_rectangle_4points(merged_pts, angle_tolerance)
    
    if best_rect_points is None:
        return None

    # 6. Return convex hull of these 4 points (reshape to Nx1x2 format)
    final_hull = cv2.convexHull(best_rect_points)
    
    return final_hull


def _find_best_rectangle_4points(points: np.ndarray, angle_tolerance: float) -> Optional[np.ndarray]:
    """
    Find the best 4 points from the given points that form the most rectangle-like quadrilateral.
    
    Args:
        points: Array of shape (N, 2) where N >= 4
        angle_tolerance: Maximum allowed deviation from 90° for corners
    
    Returns:
        Best 4 points as (4, 2) array, or None if no good rectangle found
    """
    from itertools import combinations
    
    n = len(points)
    
    # If exactly 4 points, check if they form a rectangle
    if n == 4:
        score = _score_rectangle(points, angle_tolerance)
        if score is not None:
            return _order_corners_clockwise(points)
        return None
    
    # If more than 4 points, try all combinations of 4 points
    best_score = float('inf')
    best_points = None
    
    for combo in combinations(range(n), 4):
        candidate = points[list(combo)]
        score = _score_rectangle(candidate, angle_tolerance)
        
        if score is not None and score < best_score:
            best_score = score
            best_points = candidate
    
    if best_points is not None:
        return _order_corners_clockwise(best_points)
    
    return None


def _score_rectangle(points: np.ndarray, angle_tolerance: float) -> Optional[float]:
    """
    Score how rectangle-like a set of 4 points is.
    Lower score = more rectangular.
    
    Returns:
        Score (float) if valid rectangle-like shape, None otherwise
    """
    if len(points) != 4:
        return None
    
    # Order points
    ordered = _order_corners_clockwise(points)
    
    def dist(p1: np.ndarray, p2: np.ndarray) -> float:
        return np.linalg.norm(p1 - p2)
    
    def angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
        """Calculate angle between two vectors in degrees."""
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-8)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        return np.degrees(np.arccos(cos_angle))
    
    # Calculate corner angles
    angles = []
    for i in range(4):
        v1 = ordered[(i - 1) % 4] - ordered[i]
        v2 = ordered[(i + 1) % 4] - ordered[i]
        angle = angle_between_vectors(v1, v2)
        angles.append(angle)
    
    # Check if all angles are within tolerance of 90°
    angle_deviations = [abs(angle - 90) for angle in angles]
    if any(dev > angle_tolerance for dev in angle_deviations):
        return None
    
    # Calculate side lengths
    sides = [dist(ordered[i], ordered[(i + 1) % 4]) for i in range(4)]
    
    # Score based on:
    # 1. How close angles are to 90° (lower is better)
    # 2. How similar opposite sides are (lower is better)
    angle_score = sum(angle_deviations)
    
    opposite_side_diff1 = abs(sides[0] - sides[2]) / max(sides[0], sides[2])
    opposite_side_diff2 = abs(sides[1] - sides[3]) / max(sides[1], sides[3])
    side_score = (opposite_side_diff1 + opposite_side_diff2) * 100  # Scale to similar range as angles
    
    total_score = angle_score + side_score
    
    return total_score
    """Order 4 corners clockwise starting from top-left."""
    # Find center
    center = np.mean(corners, axis=0)
    
    # Calculate angles from center
    angles = np.arctan2(corners[:, 1] - center[1], corners[:, 0] - center[0])
    
    # Sort by angle (clockwise)
    sorted_indices = np.argsort(angles)
    ordered = corners[sorted_indices]
    
    # Ensure we start from top-left (smallest x+y sum typically)
    sums = ordered[:, 0] + ordered[:, 1]
    min_idx = np.argmin(sums)
    
    # Rotate array to start from top-left
    return np.roll(ordered, -min_idx, axis=0)


def _merge_nearby_points(points: np.ndarray, threshold: float) -> np.ndarray:
    """
    Merge points that are closer than threshold distance.
    When points are merged, their centroid is used.
    
    Args:
        points: Array of shape (N, 2)
        threshold: Distance threshold for merging
    
    Returns:
        Array of merged points
    """
    if len(points) == 0:
        return points
    
    merged = []
    used = set()
    
    for i in range(len(points)):
        if i in used:
            continue
        
        # Find all points close to point i
        cluster = [points[i]]
        used.add(i)
        
        for j in range(i + 1, len(points)):
            if j in used:
                continue
            
            dist = np.linalg.norm(points[i] - points[j])
            if dist < threshold:
                cluster.append(points[j])
                used.add(j)
        
        # Use centroid of cluster
        centroid = np.mean(cluster, axis=0)
        merged.append(centroid)
    
    return np.array(merged, dtype=np.float32)


def _check_rectangle_properties(
    corners: np.ndarray, 
    angle_tolerance: float, 
    side_ratio_tolerance: float
) -> Tuple[bool, list, list]:
    """
    Check if 4 corners form a rectangle by validating:
    1. All angles are approximately 90°
    2. Opposite sides have similar lengths
    """
    
    def dist(p1: np.ndarray, p2: np.ndarray) -> float:
        return np.linalg.norm(p1 - p2)
    
    def angle_between_vectors(v1: np.ndarray, v2: np.ndarray) -> float:
        """Calculate angle between two vectors in degrees."""
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        return np.degrees(np.arccos(cos_angle))
    
    # Calculate side lengths
    sides = [dist(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    
    # Check opposite sides similarity
    # sides[0] vs sides[2], sides[1] vs sides[3]
    ratio_01 = abs(sides[0] - sides[2]) / max(sides[0], sides[2])
    ratio_23 = abs(sides[1] - sides[3]) / max(sides[1], sides[3])
    
    sides_ok = ratio_01 < side_ratio_tolerance and ratio_23 < side_ratio_tolerance
    
    # Calculate corner angles
    corner_angles = []
    for i in range(4):
        # Vectors from corner i to its neighbors
        v1 = corners[(i - 1) % 4] - corners[i]
        v2 = corners[(i + 1) % 4] - corners[i]
        corner_angle = angle_between_vectors(v1, v2)
        corner_angles.append(corner_angle)
    
    # Check if all angles are approximately 90°
    angles_ok = all(abs(angle - 90) < angle_tolerance for angle in corner_angles)
    
    is_rect = sides_ok and angles_ok
    
    return is_rect, corner_angles, [ratio_01, ratio_23]


def _visualize(
    frame: np.ndarray, 
    corners: np.ndarray, 
    is_rect: bool, 
    title: str
):
    """Visualize the detected corners on the frame."""
    vis = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2RGB)
    
    plt.figure(figsize=(8, 8))
    plt.imshow(vis)
    
    # Ensure corners is 2D (handle both (N,2) and (N,1,2) shapes from OpenCV)
    if corners.ndim == 3:
        corners = corners.reshape(-1, 2)
    
    # Draw polygon connecting corners
    if len(corners) >= 2:
        corners_plot = np.vstack([corners, corners[0:1]])  # Close the polygon
        plt.plot(corners_plot[:, 0], corners_plot[:, 1], 
                'g-' if is_rect else 'r-', linewidth=2, label='Detected Shape')
    
    # Draw corner points with different colors
    colors = ['red', 'green', 'blue', 'yellow']
    for i, (x, y) in enumerate(corners):
        plt.scatter(x, y, c=colors[i % 4], s=100, edgecolors='black', linewidths=2, zorder=5)
        plt.text(x + 10, y - 10, f"P{i}", color='white', fontsize=12, 
                weight='bold', bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))
    
    plt.title(title, fontsize=14, weight='bold')
    plt.axis('off')
    plt.legend(loc='upper right')
    plt.tight_layout()
    plt.show()



def _find_rectangle_corner(
    p1: np.ndarray, 
    p2: np.ndarray, 
    p_opp: np.ndarray
) -> Optional[np.ndarray]:
    """
    Find the corner point that forms a rectangle given two adjacent corners
    and the opposite corner.
    
    The correct corner is at the intersection of:
    - Line through p1 perpendicular to (p1 -> p_opp)
    - Line through p2 perpendicular to (p2 -> p_opp)
    
    Args:
        p1: First adjacent corner
        p2: Second adjacent corner  
        p_opp: Opposite corner
    
    Returns:
        Intersection point or None if lines are parallel
    """
    # Vector from p1 to opposite corner
    v1_to_opp = p_opp - p1
    # Perpendicular vector (rotate 90°)
    perp1 = np.array([-v1_to_opp[1], v1_to_opp[0]])
    
    # Vector from p2 to opposite corner
    v2_to_opp = p_opp - p2
    # Perpendicular vector
    perp2 = np.array([-v2_to_opp[1], v2_to_opp[0]])
    
    # Find intersection of two lines:
    # Line 1: p1 + t * perp1
    # Line 2: p2 + s * perp2
    # Solve: p1 + t * perp1 = p2 + s * perp2
    
    # Set up linear system: [perp1, -perp2] * [t, s]^T = p2 - p1
    A = np.column_stack([perp1, -perp2])
    b = p2 - p1
    
    try:
        # Solve for t and s
        params = np.linalg.solve(A, b)
        t = params[0]
        
        # Calculate intersection point
        intersection = p1 + t * perp1
        return intersection
    except np.linalg.LinAlgError:
        # Lines are parallel, no intersection
        return None


def _correct_worst_corner(
    corners: np.ndarray, 
    contour: np.ndarray, 
    angles: list
) -> Optional[np.ndarray]:
    """
    Find the corner with the worst angle deviation from 90° and attempt to correct it
    by finding the intersection of the two sides adjacent to it.
    
    Args:
        corners: 4 corners in clockwise order
        contour: Original contour
        angles: List of 4 angles at each corner
    
    Returns:
        Corrected corners if successful and within contour, None otherwise
    """
    # Find corner with worst angle (furthest from 90°)
    angle_deviations = [abs(angle - 90) for angle in angles]
    worst_idx = np.argmax(angle_deviations)
    
    # Get the three corners: prev, worst, next
    n = len(corners)
    prev_idx = (worst_idx - 1) % n
    next_idx = (worst_idx + 1) % n
    
    p_prev = corners[prev_idx]
    p_worst = corners[worst_idx]
    p_next = corners[next_idx]
    
    # Get the opposite corner (for creating perpendicular lines)
    opp_idx = (worst_idx + 2) % n
    p_opp = corners[opp_idx]
    
    # Find intersection of two lines:
    # Line 1: through p_prev, perpendicular to (p_prev -> p_opp)
    # Line 2: through p_next, perpendicular to (p_next -> p_opp)
    corrected_point = _find_rectangle_corner(p_prev, p_next, p_opp)
    
    if corrected_point is None:
        return None
    
    # Check if corrected point is within the original contour
    result = cv2.pointPolygonTest(contour, tuple(corrected_point.astype(float)), False)
    if result < 0:  # Point is outside contour
        return None
    
    # Create new corners array with corrected point
    corrected_corners = corners.copy()
    corrected_corners[worst_idx] = corrected_point
    
    return corrected_corners


def _calculate_corner_angles(corners: np.ndarray) -> list:
    """Calculate angles at each corner."""
    angles = []
    
    for i in range(4):
        p1 = corners[(i - 1) % 4]
        p2 = corners[i]
        p3 = corners[(i + 1) % 4]
        
        v1 = p1 - p2
        v2 = p3 - p2
        
        angle = np.degrees(np.arccos(
            np.clip(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)), -1, 1)
        ))
        angles.append(angle)
    
    return angles


def correctRectangleContour(
    cnt: np.ndarray,
    epsilon_factor: float = 0.02,
    merge_distance_factor: float = 0.05
) -> Optional[np.ndarray]:
    """
    Takes a contour and corrects it to form a better rectangle by:
    1. Approximating to 4 corners
    2. Finding the corner with the worst angle
    3. Correcting that corner to form proper 90° angles
    
    Args:
        cnt: Input contour (must approximate to 4 corners)
        epsilon_factor: Approximation accuracy as fraction of perimeter (0.01-0.05 typical)
        merge_distance_factor: Points closer than this fraction of perimeter are merged
    
    Returns:
        Corrected contour in OpenCV format (4, 1, 2) or None if correction fails
    """
    if len(cnt) < 4:
        return None
    
    # Approximate the contour to a polygon
    perimeter = cv2.arcLength(cnt, True)
    epsilon = epsilon_factor * perimeter
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    
    # Reshape to 2D array of points
    points = approx.reshape(-1, 2).astype(np.float32)
    
    # Merge nearby points
    merge_distance = merge_distance_factor * perimeter
    merged_points = _merge_nearby_points(points, merge_distance)
    
    # Check if we got exactly 4 points after merging
    if len(merged_points) != 4:
        return None
    
    corners = merged_points
    
    # Order corners clockwise from top-left
    corners = _order_corners_clockwise(corners)
    
    # Calculate angles to find the worst corner
    angles = _calculate_corner_angles(corners)
    
    # Try to correct the worst corner
    corrected_corners = _correct_worst_corner(corners, cnt, angles)
    
    if corrected_corners is None:
        # Return original corners if correction fails
        corrected_corners = corners
    
    # Convert to OpenCV contour format (N, 1, 2)
    corrected_contour = corrected_corners.reshape(-1, 1, 2).astype(np.int32)
    
    return corrected_contour



# Example usage
if __name__ == "__main__":
    # Create a test image with a rectangle
    img = np.ones((400, 600, 3), dtype=np.uint8) * 255
    
    # Draw a slightly rotated rectangle
    rect_corners = np.array([
        [100, 100],
        [400, 120],
        [380, 300],
        [80, 280]
    ], dtype=np.int32)
    
    cv2.fillPoly(img, [rect_corners], (100, 150, 200))
    
    # Find contours
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        cnt = contours[0]
        is_rect, corners = isRectangleFromApproximation(cnt, img, viz=True)
        print(f"Is rectangle: {is_rect}")
        if corners is not None:
            print(f"Corners:\n{corners}")