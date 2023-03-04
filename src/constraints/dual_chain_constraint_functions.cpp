#include "constraints/dual_chain_constraint_functions.h"

void DualChainConstraintsFunctions::setNames(const std::string & name1, const std::string & name2)
{
  names_[0] = name1;
  names_[1] = name2;

  n_ = 0;
  for (int i=0; i<2; ++i)
  {
    q_lengths_[i] = robot_models_[names_[i]]->getNumJoints();
    n_ += q_lengths_[i];
  }
  std::cout << names_[0] << " and " << names_[1] << std::endl
  << "q len: " << q_lengths_[0] << " and " << q_lengths_[1] << std::endl;

}

void DualChainConstraintsFunctions::setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat){
  chain_transform_ = vectorsToIsometry(pos,quat);
}

void DualChainConstraintsFunctions::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out)
{
  auto model1 = robot_models_[names_[0]];
  auto model2 = robot_models_[names_[1]];
  const Eigen::Ref<const Eigen::VectorXd> q1 = x.head(q_lengths_[0]);
  const Eigen::Ref<const Eigen::VectorXd> q2 = x.tail(q_lengths_[1]); 
  auto t1 = model1->forwardKinematics(q1);
  auto t2 = model2->forwardKinematics(q2);
  
  const Eigen::Isometry3d & current_chain = t1.inverse() * t2;  

  Eigen::Quaterniond current_quat(current_chain.linear());
  Eigen::Quaterniond init_quat(chain_transform_.linear());

  double err_r = current_quat.angularDistance(init_quat);
  double err_p = (current_chain.translation() - chain_transform_.translation()).norm();
  
  out[0] = err_p + err_r * rot_error_ratio_;
}

void DualChainConstraintsFunctions::setRotErrorRatio(double ratio)
{
  rot_error_ratio_ = ratio;
}


////
DualChainConstraintsFunctions6D::DualChainConstraintsFunctions6D()
{
  m_ = 6;
}

void DualChainConstraintsFunctions6D::setNames(const std::string & name1, const std::string & name2)
{
  names_[0] = name1;
  names_[1] = name2;

  n_ = 0;
  for (int i=0; i<2; ++i)
  {
    q_lengths_[i] = robot_models_[names_[i]]->getNumJoints();
    n_ += q_lengths_[i];
  }
  std::cout << names_[0] << " and " << names_[1] << std::endl
  << "q len: " << q_lengths_[0] << " and " << q_lengths_[1] << std::endl;

}

void DualChainConstraintsFunctions6D::setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat){
  chain_transform_ = vectorsToIsometry(pos,quat);
}

void DualChainConstraintsFunctions6D::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out)
{
  auto model1 = robot_models_[names_[0]];
  auto model2 = robot_models_[names_[1]];
  const Eigen::Ref<const Eigen::VectorXd> q1 = x.head(q_lengths_[0]);
  const Eigen::Ref<const Eigen::VectorXd> q2 = x.tail(q_lengths_[1]); 
  auto t1 = model1->forwardKinematics(q1);
  auto t2 = model2->forwardKinematics(q2);
  
  Eigen::Isometry3d chain_error = chain_transform_.inverse() * t1.inverse() * t2;
  Eigen::Matrix3d llg = chain_error.linear().log();
  // Eigen::Vector3d euler_angles = chain_error.linear().eulerAngles(2,1,0);
  out[0] = chain_error.translation()(0);
  out[1] = chain_error.translation()(1);
  out[2] = chain_error.translation()(2);
  out[3] = llg(2,1);
  out[4] = llg(0,2);
  out[5] = llg(1,0);
  
  // out[3] = euler_angles(0);
  // out[4] = euler_angles(1);
  // out[5] = euler_angles(2);

  
  // out[0] = chain_error.translation()(0) * chain_error.translation()(0);
  // out[1] = chain_error.translation()(1) * chain_error.translation()(1);
  // out[2] = chain_error.translation()(2) * chain_error.translation()(2);
  // out[3] = llg(2,1) * llg(2,1);
  // out[4] = llg(0,2) * llg(0,2);
  // out[5] = llg(1,0) * llg(1,0);

  // Eigen::Quaterniond current_quat(current_chain.linear());
  // Eigen::Quaterniond init_quat(chain_transform_.linear());

  // double err_r = current_quat.angularDistance(init_quat);
  // double err_p = (current_chain.translation() - chain_transform_.translation()).norm();
  
  // out[0] = err_p + err_r * rot_error_ratio_;
}

void DualChainConstraintsFunctions6D::setRotErrorRatio(double ratio)
{
  rot_error_ratio_ = ratio;
}
