#pragma once

#include "constraints/kinematics_constraint_functions.h"

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
