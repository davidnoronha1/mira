import cv2
import numpy as np

def correct_rectangle_contour(contours, epsilon_factor=0.02, merge_distance=10, angle_threshold=25):
	"""
	Takes a list of CV2 contours, merges them, and corrects non-90-degree angles.
	
	Parameters:
	- contours: List of CV2 contour arrays
	- epsilon_factor: Factor for contour approximation (default 0.02)
	- merge_distance: Distance threshold to merge nearby points (default 10)
	- angle_threshold: Degrees away from 90 to consider a point bad (default 25)
	
	Returns:
	- corrected_contour: numpy array of 4 corner points forming a rectangle
	"""
	
	# Step 1: Merge all contours into a single point cloud
	all_points = []
	for contour in contours:
		if contour is None:
			continue
		points = contour.reshape(-1, 2)
		all_points.extend(points)
	
	all_points = np.array(all_points, dtype=np.float32)
	
	if len(all_points) == 0:
		return np.array([])
	
	# Create a single merged contour from all points
	merged_contour = all_points.reshape(-1, 1, 2).astype(np.int32)
	
	# Step 2: Approximate the merged contour into a curve
	perimeter = cv2.arcLength(merged_contour, True)
	epsilon = epsilon_factor * perimeter
	approx = cv2.approxPolyDP(merged_contour, epsilon, True)
	
	# Convert to 2D array of points
	points = approx.reshape(-1, 2).astype(np.float32)
	
	# Step 3: Merge nearby points of the edge
	merged_points = merge_nearby_points(points, merge_distance)
	
	# Ensure we have exactly 4 points
	if len(merged_points) > 4:
		merged_points = reduce_to_four_corners(merged_points)
	elif len(merged_points) < 4:
		return merged_points.reshape(-1, 1, 2).astype(np.int32)
	
	# Step 4: Check angles and fix the bad point
	corrected_points = fix_non_90_degree_corner(merged_points, angle_threshold)
	
	return corrected_points.reshape(-1, 1, 2).astype(np.int32)


def merge_nearby_points(points, threshold):
	"""Merge points that are close to each other."""
	if len(points) <= 1:
		return points
	
	merged = []
	used = set()
	
	for i, point in enumerate(points):
		if i in used:
			continue
		
		# Find all points close to this one
		cluster = [point]
		for j, other_point in enumerate(points):
			if j != i and j not in used:
				dist = np.linalg.norm(point - other_point)
				if dist < threshold:
					cluster.append(other_point)
					used.add(j)
		
		# Average the cluster
		merged.append(np.mean(cluster, axis=0))
		used.add(i)
	
	return np.array(merged)


def reduce_to_four_corners(points):
	"""Reduce points to 4 corners using convex hull and angular spacing."""
	hull = cv2.convexHull(points.reshape(-1, 1, 2).astype(np.float32))
	hull_points = hull.reshape(-1, 2)
	
	if len(hull_points) == 4:
		return hull_points
	
	# Find the 4 points with maximum angular separation
	centroid = np.mean(hull_points, axis=0)
	angles = np.arctan2(hull_points[:, 1] - centroid[1], 
						hull_points[:, 0] - centroid[0])
	
	# Sort by angle
	sorted_idx = np.argsort(angles)
	sorted_points = hull_points[sorted_idx]
	
	# Select 4 points with roughly equal angular spacing
	n = len(sorted_points)
	indices = [int(i * n / 4) for i in range(4)]
	return sorted_points[indices]


def calculate_angle(p1, vertex, p2):
	"""Calculate the angle at vertex formed by p1-vertex-p2 in degrees."""
	v1 = p1 - vertex
	v2 = p2 - vertex
	
	# Calculate angle using dot product
	cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
	# Clamp to avoid numerical errors
	cos_angle = np.clip(cos_angle, -1.0, 1.0)
	angle = np.arccos(cos_angle)
	
	return np.degrees(angle)


def fix_non_90_degree_corner(points, angle_threshold):
	"""
	Find the corner that isn't near 90 degrees and replace it with expected location.
	"""
	if len(points) != 4:
		return points
	
	# Calculate angles at each corner
	angles = []
	for i in range(4):
		prev_point = points[(i - 1) % 4]
		current_point = points[i]
		next_point = points[(i + 1) % 4]
		
		angle = calculate_angle(prev_point, current_point, next_point)
		angles.append(angle)
	
	# Find the corner that's furthest from 90 degrees
	deviations = [abs(angle - 90) for angle in angles]
	worst_idx = np.argmax(deviations)
	
	# If the worst corner is too far from 90 degrees, fix it
	if deviations[worst_idx] > angle_threshold:
		# Get the three good points
		good_indices = [i for i in range(4) if i != worst_idx]
		p1, p2, p3 = points[good_indices]
		
		# Calculate expected position using parallelogram property
		# The fourth point completes the parallelogram
		expected = p1 + p3 - p2
		
		# Replace the bad point
		corrected = points.copy()
		corrected[worst_idx] = expected
		return corrected
	
	return points


# Example usage:
if __name__ == "__main__":
	# Create sample contours forming a rectangle with one bad corner
	contour1 = np.array([
		[[100, 100]],
		[[400, 100]]
	], dtype=np.int32)
	
	contour2 = np.array([
		[[400, 100]],
		[[450, 380]]  # This corner is off - not 90 degrees
	], dtype=np.int32)
	
	contour3 = np.array([
		[[450, 380]],
		[[100, 400]]
	], dtype=np.int32)
	
	contour4 = np.array([
		[[100, 400]],
		[[100, 100]]
	], dtype=np.int32)
	
	contours = [contour1, contour2, contour3, contour4]
	
	corrected = correct_rectangle_contour(contours)
	print(f"Number of input contours: {len(contours)}")
	print(f"Corrected rectangle (4 corners):\n{corrected.reshape(-1, 2)}")
	
	# Verify angles in corrected rectangle
	pts = corrected.reshape(-1, 2)
	for i in range(4):
		prev_pt = pts[(i - 1) % 4]
		curr_pt = pts[i]
		next_pt = pts[(i + 1) % 4]
		
		v1 = prev_pt - curr_pt
		v2 = next_pt - curr_pt
		cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
		angle = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
		print(f"Corner {i}: {angle:.2f}°")