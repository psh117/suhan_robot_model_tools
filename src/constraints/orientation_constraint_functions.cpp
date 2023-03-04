#include "constraints/orientation_constraint_functions.h"


OreintationConstraintFunctions::OreintationConstraintFunctions()
{
  // TODO Auto-generated constructor stub

}
void OreintationConstraintFunctions::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                              Eigen::Ref<Eigen::VectorXd> out)
{
  auto model = robot_models_[name_];
  auto t1 = model->forwardKinematics(x);
//   Eigen::Vector3d orientation = transform.linear() * orientaition_vector_;
//   out = orientation;
}

void OreintationConstraintFunctions::setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector)
{
  orientaition_vector_ = orientation_vector;
}

void OreintationConstraintFunctions::setName(const std::string & name)
{
  name_ = name;
}
