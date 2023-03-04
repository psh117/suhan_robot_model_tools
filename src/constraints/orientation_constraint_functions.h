#pragma once

#include "constraints/kinematics_constraint_functions.h"

class OrientationConstraintFunctions : public KinematicsConstraintsFunctions
{
public:
  OrientationConstraintFunctions();
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;
  void setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector);
  void setOrientationOffset(const Eigen::Ref<const Eigen::Matrix3d> &orientation_offset);
  void setName(const std::string & name);

private:
  Eigen::Vector3d orientaition_vector_;
  Eigen::Matrix3d orientation_offset_;
  std::string name_;
  int q_length_;
  int axis_;
};