#ifndef GEOMETRY_EIGEN_CONVERSIONS_H
#define GEOMETRY_EIGEN_CONVERSIONS_H

#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace happymoon_control {

class GeometryEigenConversions {

public:
  GeometryEigenConversions();
  ~GeometryEigenConversions();

  // Quaternions
  Eigen::Quaterniond geometryToEigen(const geometry_msgs::msg::Quaternion &vec_ros);
  geometry_msgs::msg::Quaternion
  eigenToGeometry(const Eigen::Quaterniond &vec_eigen);

  // Vectors and Points
  Eigen::Vector3d geometryToEigen(const geometry_msgs::msg::Vector3 &vec_ros);
  Eigen::Vector3d geometryToEigen(const geometry_msgs::msg::Point &vec_ros);
  geometry_msgs::msg::Vector3 eigenToGeometry(const Eigen::Vector3d &vec_eigen);
  geometry_msgs::msg::Point vectorToPoint(const geometry_msgs::msg::Vector3 &vector);
  // Pose
  Eigen::Affine3d geometryToEigen(const geometry_msgs::msg::Pose &pose_ros);

private:
};

} // namespace happymoon_control

#endif
