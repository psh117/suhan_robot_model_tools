#pragma once

#include "constraints/kinematics_constraint_functions.h"

class OrientationConstraintFunctions : public KinematicsConstraintsFunctions
{
public:
  OrientationConstraintFunctions(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
  void setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector);
  void setOrientationOffset(const Eigen::Ref<const Eigen::Matrix3d> &orientation_offset);
  void setName(const std::string & name);

protected:
  Eigen::Vector3d orientaition_vector_;
  Eigen::Matrix3d orientation_offset_;
  std::string name_;
  int q_length_ {0};
  int axis_ {0};
};

class OrientationConstrainedIK : public OrientationConstraintFunctions
{
public:
  OrientationConstrainedIK(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
  void setTargetPosition(const Eigen::Ref<const Eigen::Vector3d> &target_position);

protected:
  Eigen::Vector3d target_position_;
};