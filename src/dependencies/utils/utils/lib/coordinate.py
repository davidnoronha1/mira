import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------
# Change of Basis Matrices (From ROS to Target)
# ---------------------------------------------------------

# ROS (FLU) to ArduSub (FRD)
M_ROS_TO_ARDUSUB = np.array([
    [ 1,  0,  0],
    [ 0, -1,  0],
    [ 0,  0, -1]
])

# ROS (FLU) to OpenCV (RDF)
M_ROS_TO_OPENCV = np.array([
    [ 0, -1,  0],
    [ 0,  0, -1],
    [ 1,  0,  0]
])

# ROS (FLU) to ZED (RUB)
M_ROS_TO_ZED = np.array([
    [ 0, -1,  0],
    [ 0,  0,  1],
    [-1,  0,  0]
])


# ---------------------------------------------------------
# Core Transformation Engine
# ---------------------------------------------------------

def _transform_pose(tvec, rvec_quat, M):
    """
    Internal helper to apply a basis change matrix to a translation and rotation.
    rvec_quat is expected as [x, y, z, w].
    """
    tvec = np.asarray(tvec, dtype=float)
    tvec_new = M @ tvec
    
    if rvec_quat is not None:
        rvec_quat = np.asarray(rvec_quat, dtype=float)
        # Convert quaternion to rotation matrix
        R_old = Rotation.from_quat(rvec_quat).as_matrix()
        # Apply basis change: R_new = M * R_old * M^T
        R_new = M @ R_old @ M.T
        rvec_new = Rotation.from_matrix(R_new).as_quat()
        return tvec_new, rvec_new
    
    return tvec_new, None


# ---------------------------------------------------------
# ArduSub <-> ROS
# ---------------------------------------------------------

def ros_to_ardusub(tvec, rvec=None):
    """Converts pose from ROS (FLU) to ArduSub (FRD)."""
    return _transform_pose(tvec, rvec, M_ROS_TO_ARDUSUB)

def ardusub_to_ros(tvec, rvec=None):
    """Converts pose from ArduSub (FRD) to ROS (FLU)."""
    # Inverse of an orthogonal matrix is its transpose
    return _transform_pose(tvec, rvec, M_ROS_TO_ARDUSUB.T)

def enu_to_ned(tvec, rvec=None):
	return _transform_pose(tvec, rvec, M_ROS_TO_ARDUSUB)

def ned_to_enu(tvec, rvec=None):
	return _transform_pose(tvec, rvec, M_ROS_TO_ARDUSUB.T)

# ---------------------------------------------------------
# OpenCV <-> ROS
# ---------------------------------------------------------

def ros_to_opencv(tvec, rvec=None):
    """Converts pose from ROS (FLU) to OpenCV (RDF)."""
    return _transform_pose(tvec, rvec, M_ROS_TO_OPENCV)

def opencv_to_ros(tvec, rvec=None):
    """Converts pose from OpenCV (RDF) to ROS (FLU)."""
    return _transform_pose(tvec, rvec, M_ROS_TO_OPENCV.T)


# ---------------------------------------------------------
# ZED Camera <-> ROS
# ---------------------------------------------------------

def ros_to_zed(tvec, rvec=None):
    """Converts pose from ROS (FLU) to ZED (RUB)."""
    return _transform_pose(tvec, rvec, M_ROS_TO_ZED)

def zed_to_ros(tvec, rvec=None):
    """Converts pose from ZED (RUB) to ROS (FLU)."""
    return _transform_pose(tvec, rvec, M_ROS_TO_ZED.T)


# ---------------------------------------------------------
# Euler & Quaternion Utilities
# ---------------------------------------------------------

def euler_from_quat(quat, degrees=False):
    """
    Converts a quaternion [x, y, z, w] to Euler angles [roll, pitch, yaw].
    Uses the 'xyz' (Roll-Pitch-Yaw) sequence standard in many robotics applications.
    """
    quat = np.asarray(quat, dtype=float)
    # Extrinsic xyz (capital letters in scipy indicate intrinsic, lowercase extrinsic)
    # Intrinsic zyx (yaw, pitch, roll) is mathematically equivalent to extrinsic xyz.
    r = Rotation.from_quat(quat)
    roll, pitch, yaw = r.as_euler('xyz', degrees=degrees)
    return roll, pitch, yaw

def quat_from_euler(roll, pitch, yaw, degrees=False):
    """
    Converts Euler angles [roll, pitch, yaw] to a quaternion [x, y, z, w].
    """
    r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=degrees)
    return r.as_quat()

# ---------------------------------------------------------
# Example Usage
# ---------------------------------------------------------
if __name__ == "__main__":
    # Example ROS pose: 1m forward, 2m left, 3m up
    ros_tvec = [1.0, 2.0, 3.0]
    # No rotation (identity quaternion)
    ros_rvec = [0.0, 0.0, 0.0, 1.0] 

    cv_tvec, cv_rvec = ros_to_opencv(ros_tvec, ros_rvec)
    print(f"ROS tvec: {ros_tvec} -> OpenCV tvec: {cv_tvec}")
    
    # Let's test Euler/Quat conversions
    r, p, y = np.pi/4, 0.0, np.pi/2
    q = quat_from_euler(r, p, y)
    print(f"\nEuler (R,P,Y): {r:.2f}, {p:.2f}, {y:.2f} -> Quat (x,y,z,w): {q}")
    
    r2, p2, y2 = euler_from_quat(q)
    print(f"Quat back to Euler: {r2:.2f}, {p2:.2f}, {y2:.2f}")