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
                                   const std::string & tip_link, const std::string& URDF_param = "/robot_description");
  TRACIKAdapter & getTRACIKAdapter(const std::string & name);
  bool project(Eigen::Ref<Eigen::VectorXd> q);
  void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out);
  virtual void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) = 0;
protected:
  std::map<std::string, TRACIKAdapterPtr> robot_models_;
  Eigen::VectorXd q_;
  unsigned int n_; ///< dim of q_
  double tolerance_{1e-5};
  int maxIterations_{1000};
};

class DualChainConstraintsFunctions : public KinematicsConstraintsFunctions
{
public:
  void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                    Eigen::Ref<Eigen::VectorXd> out) override;

  void setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void setNames(const std::string & name1, const std::string & name2);

private:
  Eigen::Isometry3d chain_transform_;
  std::array<std::string, 2> names_;
  std::array<int,2> q_lengths_;

};

