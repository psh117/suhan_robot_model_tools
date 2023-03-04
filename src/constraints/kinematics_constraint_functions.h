#pragma once

#include <map>
#include <array>
#include <string>
#include <Eigen/Dense>
#include <ompl/base/Constraint.h>
#include "trac_ik_adapter/trac_ik_adapter.h"
#include "eigen_tools/eigen_tools.h"

const std::string cgreen{"\033[0;32m"};
const std::string creset{"\033[0m"};

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
