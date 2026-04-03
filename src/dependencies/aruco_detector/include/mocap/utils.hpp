#include "opencv2/core/matx.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Transform.h>

void quaternionFromRvecs(const cv::Vec3d &rvec, tf2::Quaternion &q);
void transformStampedToTransform(
    const geometry_msgs::TransformStamped &msg, tf2::Transform &tf);
void transformToTransformStamped(
    const tf2::Transform &tf, geometry_msgs::TransformStamped &msg);