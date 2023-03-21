#pragma once

#include "constraints/kinematics_constraint_functions.h"

class MultiChainConstraintFunctions : public KinematicsConstraintsFunctions
{
public:
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;
  void setChainsFromJoints(const Eigen::Ref<const Eigen::VectorXd> &q);
  void setChains(const Eigen::Ref<const Eigen::VectorXd> &poses);
  // void setNames(const std::string & name1, const std::string & name2, const std::string & name3);
  void setNames(const std::vector<std::string> & names);
  void setRotErrorRatio(double ratio);
  
protected:
  std::vector<Eigen::Isometry3d> chain_transform_;
  std::vector<std::string> names_;
  std::vector<int> q_lengths_;
  double rot_error_ratio_ {1.0};
};


class MultiChainConstraintIK : public MultiChainConstraintFunctions
{
public:
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;
  void setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void setNames(const std::vector<std::string> & names);
  
protected:
  Eigen::Isometry3d target_pose_;
};