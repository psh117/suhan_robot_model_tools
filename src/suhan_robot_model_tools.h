#pragma once

#include <map>
#include <array>
#include <string>
#include <Eigen/Dense>
#include <ompl/base/Constraint.h>
#include "trac_ik_adapter/trac_ik_adapter.h"
#include "eigen_tools/eigen_tools.h"

class KinematicsConstraintsFunctions
{
public:
  TRACIKAdapter & addTRACIKAdapter(const std::string & name, const std::string & base_link, 
                                   const std::string & tip_link, double max_time, double precision, const std::string& URDF_param = "/robot_description");
  TRACIKAdapter & getTRACIKAdapter(const std::string & name);
  bool project(Eigen::Ref<Eigen::VectorXd> q);
  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out);
  virtual void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) = 0;
  void setTolerance(double tolerance);
  void setMaxIterations(int maxIterations);
  void setNumFiniteDiff(int num_finite_diff);
protected:
  std::map<std::string, TRACIKAdapterPtr> robot_models_;
  Eigen::VectorXd q_;
  unsigned int n_; ///< dim of q_
  unsigned int m_ {1}; ///< dim of constraint
  int num_finite_diff_ {7};
  double tolerance_{1e-3};
  int maxIterations_{1000};

};

class DualChainConstraintsFunctions : public KinematicsConstraintsFunctions
{
public:
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;

  void setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void setNames(const std::string & name1, const std::string & name2);
  void setRotErrorRatio(double ratio);
  Eigen::MatrixXd getJacobian6d(const Eigen::Ref<const Eigen::VectorXd> &x);
private:
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
  Eigen::MatrixXd getJacobian6d(const Eigen::Ref<const Eigen::VectorXd> &x);
private:
  Eigen::Isometry3d chain_transform_;
  std::array<std::string, 2> names_;
  std::array<int,2> q_lengths_;
  double rot_error_ratio_ {1.0};

};

class TripleChainConstraintsFunctions : public KinematicsConstraintsFunctions
{
public:
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;
  void setChain(const Eigen::Ref<const Eigen::VectorXd> &q1, 
                const Eigen::Ref<const Eigen::VectorXd> &q2, 
                const Eigen::Ref<const Eigen::VectorXd> &q3);
  void setNames(const std::string & name1, const std::string & name2, const std::string & name3);
  void setRotErrorRatio(double ratio);
  
private:
  std::array<Eigen::Isometry3d, 2> chain_transform_;
  std::array<std::string, 3> names_;
  std::array<int,3> q_lengths_;
  double rot_error_ratio_ {1.0};

};

