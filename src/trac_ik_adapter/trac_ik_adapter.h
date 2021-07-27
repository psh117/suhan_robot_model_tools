#pragma once

#include <mutex>
#include <random>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <trac_ik/trac_ik.hpp>

Eigen::Matrix3d getEigenRotation(const KDL::Rotation & r);
Eigen::Vector3d getEigenVector(const KDL::Vector& v);
Eigen::Isometry3d getEigenFrame(const KDL::Frame & frame);

KDL::Rotation getKDLRotation(const Eigen::Matrix3d & matrix);
KDL::Vector getKDLVector(const Eigen::Vector3d& vector);
KDL::Frame getKDLFrame(const Eigen::Isometry3d & transform);

class TRACIKAdapter
{
public:
  TRACIKAdapter(const std::string& base_link, const std::string& tip_link, double max_time, double precision, const std::string& URDF_param = "/robot_description");
  bool solve(const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution);
  bool solve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution);
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &q);

  Eigen::VectorXd getRandomConfig();
  Eigen::VectorXd getGaussianRandomConfig(const Eigen::Ref<const Eigen::VectorXd> &q0, double sigma = 0.1);
  Eigen::VectorXd getGaussianRandomConfig(double sigma);
  Eigen::VectorXd getStrictRandomConfig(double margin);
  bool randomSolveWithStartOnce(const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution);
  bool randomSolve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution);
  bool randomGaussianSolve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution, double sigma);
  bool randomStrictSolve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution, double margin);
  Eigen::VectorXd getLowerBound() const { return lb_; }
  Eigen::VectorXd getUpperBound() const { return ub_; }
  void setBounds(const Eigen::Ref<const Eigen::VectorXd> &lb, const Eigen::Ref<const Eigen::VectorXd> &ub);
  void setToleranceBounds(double px, double py, double pz, double ox, double oy, double oz);
  unsigned int getNumJoints() const { return n_joint_; }
  bool isValid(const Eigen::Ref<const Eigen::VectorXd> &q);

protected:
  unsigned int n_joint_ {0};
  KDL::JntArray nominal_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;

  std::default_random_engine rg_;

  Eigen::Isometry3d last_transform_;
  std::mutex iK_solver_mutex_;

  TRAC_IK::TRAC_IK trac_ik_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::Chain chain_;
  KDL::Twist bounds_;
};


typedef std::shared_ptr<TRACIKAdapter> TRACIKAdapterPtr;