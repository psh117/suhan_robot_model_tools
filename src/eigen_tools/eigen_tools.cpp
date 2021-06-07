#include "eigen_tools.h"

Eigen::Isometry3d vectorsToIsometry(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform.linear() = Eigen::Quaterniond(quat).toRotationMatrix();
  transform.translation() = pos;

  return transform;
}

std::pair<Eigen::Vector3d, Eigen::Vector4d> isometryToVectors(const Eigen::Isometry3d & transform)
{
  Eigen::Vector3d pos = transform.translation();
  Eigen::Vector4d quat = Eigen::Quaterniond(transform.linear()).coeffs();

  return std::make_pair(pos, quat);
}