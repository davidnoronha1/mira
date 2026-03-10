import numpy as np

def find_rectangle_corners(points: np.ndarray):
	"""Find the four corners of a rectangle from a set of points."""
	points = points.astype(np.float32)
	
	# Calculate sums and differences for each point
	sums = points[:, 0] + points[:, 1]  # x + y
	diffs = points[:, 0] - points[:, 1]  # x - y
	
	# Find corner indices
	top_left_idx = np.argmin(sums)      # smallest x + y
	bottom_right_idx = np.argmax(sums)   # largest x + y
	top_right_idx = np.argmax(diffs)     # largest x - y
	bottom_left_idx = np.argmin(diffs)   # smallest x - y
	
	corners = {
		'top_left': points[top_left_idx],
		'top_right': points[top_right_idx],
		'bottom_left': points[bottom_left_idx],
		'bottom_right': points[bottom_right_idx]
	}
	
	return corners