#include "geometry_msgs/msg/transform_stamped.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/matx.hpp"
#include <tf2/LinearMath/Transform.h>

// In mocap/utils.hpp or directly in your .cpp file
void quaternionFromRvecs(const cv::Vec3d &rvec,
             tf2::Quaternion &q) { // Changed Vec3f to Vec3d
  cv::Matx33d rmat;
  cv::Rodrigues(rvec, rmat);

  tf2::Quaternion rqtn;
  tf2::Matrix3x3(rmat(0, 0), rmat(0, 1), rmat(0, 2), rmat(1, 0), rmat(1, 1),
         rmat(1, 2), rmat(2, 0), rmat(2, 1), rmat(2, 2))
    .getRotation(rqtn);

  q = rqtn.normalized(); // Keep this line as a safeguard
}

void transformStampedToTransform(const geometry_msgs::msg::TransformStamped &msg,
                 tf2::Transform &tf) {
  tf2::Vector3 translation(msg.transform.translation.x,
               msg.transform.translation.y,
               msg.transform.translation.z);
  tf2::Quaternion rotation(msg.transform.rotation.x, msg.transform.rotation.y,
               msg.transform.rotation.z, msg.transform.rotation.w);
  tf.setOrigin(translation);
  tf.setRotation(rotation);
}

void transformToTransformStamped(const tf2::Transform &tf,
                 geometry_msgs::msg::TransformStamped &msg) {
  msg.transform.translation.x = tf.getOrigin().x();
  msg.transform.translation.y = tf.getOrigin().y();
  msg.transform.translation.z = tf.getOrigin().z();

  msg.transform.rotation.x = tf.getRotation().x();
  msg.transform.rotation.y = tf.getRotation().y();
  msg.transform.rotation.z = tf.getRotation().z();
  msg.transform.rotation.w = tf.getRotation().w();
}