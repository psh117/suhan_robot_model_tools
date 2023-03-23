#include "constraints/multi_chain_constraint_functions.h"


void MultiChainConstraintFunctions::setNames(const std::vector<std::string> & names)
{
  names_ = names;

  int robot_num = names_.size();
  q_lengths_.resize(robot_num);

  n_ = 0;
  m_ = 6 * (robot_num - 1);

  for (int i=0; i<robot_num; ++i)
  {
    q_lengths_[i] = robot_models_[names_[i]]->getNumJoints();
    n_ += q_lengths_[i];
  }
  
  for (int i=0; i<robot_num; ++i)
  {
    std::cout << names_[i] << " and " << q_lengths_[i] << std::endl;
  }

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
    std::cout << cred << "[error] q size is not correct" << creset << std::endl;
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
                                  Eigen::Ref<Eigen::VectorXd> out)
{
  auto & model0 = robot_models_[names_[0]];
  auto t0 = model0->forwardKinematics(x.segment(0, q_lengths_[0]));

  int cur_idx = q_lengths_[0];
  for (int i=0; i<chain_transform_.size(); ++i)
  {
    auto & model = robot_models_[names_[i+1]];
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

void MultiChainConstraintIK::setNames(const std::vector<std::string> & names)
{
  MultiChainConstraintFunctions::setNames(names);
  m_ = 6 * (names.size()); // IK task added
}

void MultiChainConstraintIK::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out)
{
  auto & model0 = robot_models_[names_[0]];
  auto t0 = model0->forwardKinematics(x.segment(0, q_lengths_[0]));

  int cur_idx = q_lengths_[0];

  for (int i=0; i<chain_transform_.size(); ++i)
  {
    auto & model = robot_models_[names_[i+1]];
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