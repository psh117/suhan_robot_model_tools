#include "constraints/multi_chain_constraint_functions.h"

MultiChainConstraintFunctions::MultiChainConstraintFunctions(const unsigned int ambientDim, const unsigned int coDim)
  : KinematicsConstraintsFunctions(ambientDim, coDim)
{
}

void MultiChainConstraintFunctions::setNames(const std::vector<std::string> & names)
{
  names_ = names;

  int robot_num = names_.size();
  q_lengths_.resize(robot_num);

  int n = 0;
  m_ = 6 * (robot_num - 1);

  for (int i=0; i<robot_num; ++i)
  {
    q_lengths_[i] = robot_models_[names_[i]]->getNumJoints();
    n += q_lengths_[i];
  }
  
  assert(n == n_);

  // for (int i=0; i<robot_num; ++i)
  // {
  //   std::cout << names_[i] << " and " << q_lengths_[i] << std::endl;
  // }

  lb_.resize(n_);
  ub_.resize(n_);
  
  int cur_idx = 0;
  
  for (int i=0; i<robot_num; ++i)
  {
    lb_.segment(cur_idx, q_lengths_[i]) = robot_models_[names_[i]]->getLowerBound();
    ub_.segment(cur_idx, q_lengths_[i]) = robot_models_[names_[i]]->getUpperBound();
    cur_idx += q_lengths_[i];
  }
}

void MultiChainConstraintFunctions::setChainsFromJoints(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  if (q.size() != n_)
  {
    std::cout << cred << "[error] q size is not correct q.size() : " << q.size() << ", n_ : " << n_ << creset << std::endl;
    return;
  }

  int robot_num = names_.size();

  chain_transform_.resize(robot_num - 1);
  auto t0 = robot_models_[names_[0]]->forwardKinematics(q.segment(0, q_lengths_[0]));
  int cur_idx = q_lengths_[0];
  
  for (int i=1; i<robot_num; ++i)
  {
    const Eigen::Ref<const Eigen::VectorXd> qi = q.segment(cur_idx, q_lengths_[i]);
    cur_idx += q_lengths_[i];
    auto ti = robot_models_[names_[i]]->forwardKinematics(qi);
    chain_transform_[i-1] = t0.inverse() * ti;
  }
  if (debug_level_ > 3)
  {
    std::cout << cgreen << "[info] chain is successfully registered"  << creset << std::endl;
    for (int i=0; i<robot_num-1; ++i)
    {
      std::cout << "[chain_transform_[" << i << "]]" << std::endl 
                << chain_transform_[i].matrix() << std::endl;
    }
  }
}

void MultiChainConstraintFunctions::setChains(const Eigen::Ref<const Eigen::VectorXd> &poses)
{
  int robot_num = names_.size();
  
  if (poses.size() != 7 * (robot_num - 1))
  {
    std::cout << cred << "[error] poses size is not correct" << creset << std::endl;
    return;
  }

  chain_transform_.resize(robot_num - 1);

  int cur_idx = 0;
  for (int i=0; i<robot_num-1; ++i)
  {
    chain_transform_[i] = vectorsToIsometry(poses.segment(cur_idx, 3), poses.segment(cur_idx+3, 4));
    cur_idx += 7;
  }

  if (debug_level_ > 3)
  {
    std::cout << cgreen << "[info] chain is successfully registered"  << creset << std::endl;
    for (int i=0; i<robot_num-1; ++i)
    {
      std::cout << "[chain_transform_[" << i << "]]" << std::endl 
                << chain_transform_[i].matrix() << std::endl;
    }
  }
}

void MultiChainConstraintFunctions::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
  auto & model0 = robot_models_.at(names_[0]);
  auto t0 = model0->forwardKinematics(x.segment(0, q_lengths_[0]));

  int cur_idx = q_lengths_[0];
  for (int i=0; i<chain_transform_.size(); ++i)
  {
    auto & model = robot_models_.at(names_[i+1]);
    auto ti = model->forwardKinematics(x.segment(cur_idx, q_lengths_[i+1]));
    
    cur_idx += q_lengths_[i+1];
    
    Eigen::Isometry3d chain_error = chain_transform_[i].inverse() * t0.inverse() * ti;
    Eigen::Matrix3d llg = chain_error.linear().log();

    out[i*6] = chain_error.translation()(0);
    out[i*6+1] = chain_error.translation()(1);
    out[i*6+2] = chain_error.translation()(2);
    out[i*6+3] = llg(2,1) * rot_error_ratio_;
    out[i*6+4] = llg(0,2) * rot_error_ratio_;
    out[i*6+5] = llg(1,0) * rot_error_ratio_;
  }
}

void MultiChainConstraintFunctions::setRotErrorRatio(double ratio)
{
  rot_error_ratio_ = ratio;
}

MultiChainConstraintIK::MultiChainConstraintIK(const unsigned int ambientDim, const unsigned int coDim) : 
  MultiChainConstraintFunctions(ambientDim, coDim)
{

}

void MultiChainConstraintIK::setNames(const std::vector<std::string> & names)
{
  MultiChainConstraintFunctions::setNames(names);
  m_ = 6 * (names.size()); // IK task added
}

void MultiChainConstraintIK::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & model0 = robot_models_.at(names_[0]);
  auto t0 = model0->forwardKinematics(x.segment(0, q_lengths_[0]));

  int cur_idx = q_lengths_[0];

  for (int i=0; i<chain_transform_.size(); ++i)
  {
    const auto & model = robot_models_.at(names_[i+1]);
    auto ti = model->forwardKinematics(x.segment(cur_idx, q_lengths_[i+1]));
    
    cur_idx += q_lengths_[i+1];
    
    Eigen::Isometry3d chain_error = chain_transform_[i].inverse() * t0.inverse() * ti;
    Eigen::Matrix3d llg = chain_error.linear().log();

    out[i*6] = chain_error.translation()(0);
    out[i*6+1] = chain_error.translation()(1);
    out[i*6+2] = chain_error.translation()(2);
    out[i*6+3] = llg(2,1) * rot_error_ratio_;
    out[i*6+4] = llg(0,2) * rot_error_ratio_;
    out[i*6+5] = llg(1,0) * rot_error_ratio_;
  }

  auto & target_error = t0.inverse() * target_pose_;
  Eigen::Matrix3d llg = target_error.linear().log();

  out[chain_transform_.size()*6] = target_error.translation()(0);
  out[chain_transform_.size()*6+1] = target_error.translation()(1);
  out[chain_transform_.size()*6+2] = target_error.translation()(2);
  out[chain_transform_.size()*6+3] = llg(2,1) * rot_error_ratio_;
  out[chain_transform_.size()*6+4] = llg(0,2) * rot_error_ratio_;
  out[chain_transform_.size()*6+5] = llg(1,0) * rot_error_ratio_;
}

void MultiChainConstraintIK::setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  target_pose_ = vectorsToIsometry(pos, quat);
}

MultiChainWithFixedOrientationConstraint::MultiChainWithFixedOrientationConstraint(const unsigned int ambientDim, const unsigned int coDim) : 
  MultiChainConstraintFunctions(ambientDim, coDim)
{

}

void MultiChainWithFixedOrientationConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
  auto & model0 = robot_models_.at(names_[0]);
  auto t0 = model0->forwardKinematics(x.segment(0, q_lengths_[0]));

  int cur_idx = q_lengths_[0];

  for (int i=0; i<chain_transform_.size(); ++i)
  {
    auto & model = robot_models_.at(names_[i+1]);
    auto ti = model->forwardKinematics(x.segment(cur_idx, q_lengths_[i+1]));
    
    cur_idx += q_lengths_[i+1];
    
    Eigen::Isometry3d chain_error = chain_transform_[i].inverse() * t0.inverse() * ti;
    Eigen::Matrix3d llg = chain_error.linear().log();

    out[i*6] = chain_error.translation()(0);
    out[i*6+1] = chain_error.translation()(1);
    out[i*6+2] = chain_error.translation()(2);
    out[i*6+3] = llg(2,1) * rot_error_ratio_;
    out[i*6+4] = llg(0,2) * rot_error_ratio_;
    out[i*6+5] = llg(1,0) * rot_error_ratio_;
  }

  Eigen::Matrix3d R = orientation_offset_ * t0.linear().transpose();
  switch (axis_)
  {
  case 0:
    out[chain_transform_.size()*6] = asin(R(1, 0));
    out[chain_transform_.size()*6+1] = asin(R(2, 0));
    break;
  case 1:
    out[chain_transform_.size()*6] = asin(R(0, 1));
    out[chain_transform_.size()*6+1] = asin(R(2, 1));
    break;
  case 2:
    out[chain_transform_.size()*6] = asin(R(0, 2));
    out[chain_transform_.size()*6+1] = asin(R(1, 2));
    break;
  default:
    break;
  }

}

void MultiChainWithFixedOrientationConstraint::setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector)
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

void MultiChainWithFixedOrientationConstraint::setOrientationOffset(const Eigen::Ref<const Eigen::Matrix3d> &orientation_offset)
{
  orientation_offset_ = orientation_offset;
}

void MultiChainWithFixedOrientationConstraint::setNames(const std::vector<std::string> & names)
{
  MultiChainConstraintFunctions::setNames(names);
  m_ += 2; // Orientation task added
}

// void MultiChainWithFixedOrientationConstraintIK::function(const Eigen::Ref<const Eigen::VectorXd> &x,
//                                   Eigen::Ref<Eigen::VectorXd> out)
// {
//   auto & model0 = robot_models_[names_[0]];
//   auto t0 = model0->forwardKinematics(x.segment(0, q_lengths_[0]));

//   int cur_idx = q_lengths_[0];

//   for (int i=0; i<chain_transform_.size(); ++i)
//   {
//     auto & model = robot_models_[names_[i+1]];
//     auto ti = model->forwardKinematics(x.segment(cur_idx, q_lengths_[i+1]));
    
//     cur_idx += q_lengths_[i+1];
    
//     Eigen::Isometry3d chain_error = chain_transform_[i].inverse() * t0.inverse() * ti;
//     Eigen::Matrix3d llg = chain_error.linear().log();

//     out[i*6] = chain_error.translation()(0);
//     out[i*6+1] = chain_error.translation()(1);
//     out[i*6+2] = chain_error.translation()(2);
//     out[i*6+3] = llg(2,1) * rot_error_ratio_;
//     out[i*6+4] = llg(0,2) * rot_error_ratio_;
//     out[i*6+5] = llg(1,0) * rot_error_ratio_;
//   }

//   Eigen::Matrix3d R = orientation_offset_.transpose() * t0.linear();
//   switch (axis_)
//   {
//   case 0:
//     out[chain_transform_.size()*6] = asin(R(1, 0));
//     out[chain_transform_.size()*6+1] = asin(R(2, 0));
//     break;
//   case 1:
//     out[chain_transform_.size()*6] = asin(R(0, 1));
//     out[chain_transform_.size()*6+1] = asin(R(2, 1));
//     break;
//   case 2:
//     out[chain_transform_.size()*6] = asin(R(0, 2));
//     out[chain_transform_.size()*6+1] = asin(R(1, 2));
//     break;
//   default:
//     break;
//   }

//   auto & target_error = t0.inverse() * target_pose_;
//   Eigen::Matrix3d llg2 = target_error.linear().log();
//   out[chain_transform_.size()*6+2] = target_error.translation()(0);
//   out[chain_transform_.size()*6+3] = target_error.translation()(1);
//   out[chain_transform_.size()*6+4] = target_error.translation()(2);
//   out[chain_transform_.size()*6+5] = llg2(2,1) * rot_error_ratio_;
//   out[chain_transform_.size()*6+6] = llg2(0,2) * rot_error_ratio_;
//   out[chain_transform_.size()*6+7] = llg2(1,0) * rot_error_ratio_;
// }