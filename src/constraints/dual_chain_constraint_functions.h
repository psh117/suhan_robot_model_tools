#pragma once

#include "constraints/kinematics_constraint_functions.h"

class DualChainConstraintsFunctions : public KinematicsConstraintsFunctions
{
public:
  DualChainConstraintsFunctions(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;

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
  DualChainConstraintsFunctions6D(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;

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
  DualChainConstraintIK(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
  void setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

private:
  Eigen::Isometry3d target_pose_;

};


class DualChainWithFixedOrientationConstraint : public DualChainConstraintsFunctions6D
{
public:
  DualChainWithFixedOrientationConstraint(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
  void setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector);
  void setOrientationOffset(const Eigen::Ref<const Eigen::Matrix3d> &orientation_offset);

protected:
  Eigen::Isometry3d chain_transform_;
  std::array<std::string, 2> names_;
  std::array<int,2> q_lengths_;
  double rot_error_ratio_ {1.0};

  Eigen::Vector3d orientaition_vector_;
  Eigen::Matrix3d orientation_offset_;
  std::string name_;
  int q_length_;
  int axis_;

};


class DualChainWithFixedOrientationConstraintIK : public DualChainWithFixedOrientationConstraint
{
public:
  DualChainWithFixedOrientationConstraintIK(const unsigned int ambientDim, const unsigned int coDim);
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) const override;
  void setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos);


protected:
  Eigen::Vector3d target_pos_;
  
};

