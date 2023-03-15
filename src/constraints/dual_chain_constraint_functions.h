#pragma once

#include "constraints/kinematics_constraint_functions.h"

class DualChainConstraintsFunctions : public KinematicsConstraintsFunctions
{
public:
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;

  void setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void setNames(const std::string & name1, const std::string & name2);
  void setRotErrorRatio(double ratio);

protected:
  Eigen::Isometry3d chain_transform_;
  std::array<std::string, 2> names_;
  std::array<int,2> q_lengths_;
  double rot_error_ratio_ {1.0};

};

class DualChainConstraintsFunctions6D : public KinematicsConstraintsFunctions
{
public:
  DualChainConstraintsFunctions6D();
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;

  void setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void setNames(const std::string & name1, const std::string & name2);
  void setRotErrorRatio(double ratio);

protected:
  Eigen::Isometry3d chain_transform_;
  std::array<std::string, 2> names_;
  std::array<int,2> q_lengths_;
  double rot_error_ratio_ {1.0};

};



class DualChainConstraintIK : public DualChainConstraintsFunctions6D
{
public:
  DualChainConstraintIK();
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;
  void setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

private:
  Eigen::Isometry3d target_pose_;

};