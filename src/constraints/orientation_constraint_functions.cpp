#include "constraints/orientation_constraint_functions.h"
#include <cmath>

OrientationConstraintFunctions::OrientationConstraintFunctions(const unsigned int ambientDim, const unsigned int coDim)
  : KinematicsConstraintsFunctions(ambientDim, coDim)
{
  // TODO Auto-generated constructor stub
  orientation_offset_.setIdentity();
  m_ = 2;

}
void OrientationConstraintFunctions::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                              Eigen::Ref<Eigen::VectorXd> out) const
{
  auto model = robot_models_.at(name_);
  auto t1 = model->forwardKinematics(x);
  Eigen::Matrix3d R = orientation_offset_.transpose() * t1.linear();
  switch (axis_)
  {
  case 0:
    out[0] = asin(R(1, 0));
    out[1] = asin(R(2, 0));
    break;
  case 1:
    out[0] = asin(R(0, 1));
    out[1] = asin(R(2, 1));
    break;
  case 2:
    out[0] = asin(R(0, 2));
    out[1] = asin(R(1, 2));
    break;
  default:
    break;
  }
}

void OrientationConstraintFunctions::setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector)
{
  // currently support only one component of orientation vector
  // x == 1, y == 1, z == 1
  if (orientation_vector[0] > 0.9)
  {
    axis_ = 0;
  }
  else if (orientation_vector[1] > 0.9)
  {
    axis_ = 1;
  }
  else if (orientation_vector[2] > 0.9)
  {
    axis_ = 2;
  }
  else
  {
    std::cout << "orientation vector is not supported" << std::endl;
  }
  
  orientaition_vector_ = orientation_vector;
}

void OrientationConstraintFunctions::setOrientationOffset(const Eigen::Ref<const Eigen::Matrix3d> &orientation_offset)
{
  orientation_offset_ = orientation_offset;
}

void OrientationConstraintFunctions::setName(const std::string & name)
{
  name_ = name;
  int n = robot_models_[name_]->getNumJoints();
  
  assert(n == n_); 

  // std::cout << name_ << " and " << "q len: " << n_ << std::endl;
  lb_ = robot_models_[name_]->getLowerBound();
  ub_ = robot_models_[name_]->getUpperBound();
}

OrientationConstrainedIK::OrientationConstrainedIK(const unsigned int ambientDim, const unsigned int coDim)
  : OrientationConstraintFunctions(ambientDim, coDim)
{
  // TODO Auto-generated constructor stub
  orientation_offset_.setIdentity();
  m_ = 5;
}

void OrientationConstrainedIK::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                        Eigen::Ref<Eigen::VectorXd> out) const
{
  auto model = robot_models_.at(name_);
  auto t1 = model->forwardKinematics(x);
  Eigen::Matrix3d R = orientation_offset_.transpose() * t1.linear();
  switch (axis_)
  {
  case 0:
    out[0] = asin(R(1, 0));
    out[1] = asin(R(2, 0));
    break;
  case 1:
    out[0] = asin(R(0, 1));
    out[1] = asin(R(2, 1));
    break;
  case 2:
    out[0] = asin(R(0, 2));
    out[1] = asin(R(1, 2));
    break;
  default:
    break;
  }
  out[2] = target_position_[0] - t1.translation()[0];
  out[3] = target_position_[1] - t1.translation()[1];
  out[4] = target_position_[2] - t1.translation()[2];
}

void OrientationConstrainedIK::setTargetPosition(const Eigen::Ref<const Eigen::Vector3d> &target_position)
{
  target_position_ = target_position;
}
