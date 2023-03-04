#pragma once

#include "constraints/kinematics_constraint_functions.h"

class OreintationConstraintFunctions : public KinematicsConstraintsFunctions
{
public:
  OreintationConstraintFunctions();
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;
  void setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector);
  void setName(const std::string & name);

private:
  Eigen::Vector3d orientaition_vector_;
  std::string name_;
  int q_length_;
};