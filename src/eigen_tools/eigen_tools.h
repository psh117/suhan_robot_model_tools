#pragma once

#include <utility>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

Eigen::Isometry3d vectorsToIsometry(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
std::pair<Eigen::Vector3d, Eigen::Vector4d> isometryToVectors(const Eigen::Isometry3d & transform);
