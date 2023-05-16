#pragma once

#include "constraints/kinematics_constraint_functions.h"

class MultiChainConstraintFunctions : public KinematicsConstraintsFunctions
{
public:
  MultiChainConstraintFunctions(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
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
  MultiChainConstraintIK(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
  void setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void setNames(const std::vector<std::string> & names);
  
protected:
  Eigen::Isometry3d target_pose_;
};


class MultiChainWithFixedOrientationConstraint : public MultiChainConstraintFunctions
{
public:
  MultiChainWithFixedOrientationConstraint(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
  void setNames(const std::vector<std::string> & names);
  void setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector);
  void setOrientationOffset(const Eigen::Ref<const Eigen::Matrix3d> &orientation_offset);

protected:
  Eigen::Vector3d orientaition_vector_;
  Eigen::Matrix3d orientation_offset_;
  int q_length_;
  int axis_;
};


// Use MultiChainConstraintIK instead of this class
// class MultiChainWithFixedOrientationConstraintIK : public MultiChainWithFixedOrientationConstraint
// {
// public:
//   void function(const Eigen::Ref<const Eigen::VectorXd> &x,
//                                     Eigen::Ref<Eigen::VectorXd> out) override;
//   void setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
//   void setNames(const std::vector<std::string> & names);

// protected:
//   Eigen::Isometry3d target_pose_;
// };