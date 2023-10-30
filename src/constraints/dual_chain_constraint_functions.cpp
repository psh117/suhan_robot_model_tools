#include "constraints/dual_chain_constraint_functions.h"
#include "dual_chain_constraint_functions.h"

DualChainConstraintsFunctions::DualChainConstraintsFunctions(const unsigned int ambientDim, const unsigned int coDim) : 
  KinematicsConstraintsFunctions(ambientDim, coDim)
{
}

void DualChainConstraintsFunctions::setNames(const std::string & name1, const std::string & name2)
{
  names_[0] = name1;
  names_[1] = name2;

  int n = 0;
  
  for (int i=0; i<2; ++i)
  {
    q_lengths_[i] = robot_models_[names_[i]]->getNumJoints();
    n += q_lengths_[i];
  }
  // std::cout << names_[0] << " and " << names_[1] << std::endl
  // << "q len: " << q_lengths_[0] << " and " << q_lengths_[1] << std::endl;

  assert(n == n_);
  lb_.resize(n_);
  ub_.resize(n_);
  int cur_idx = 0;
  for (int i=0; i<2; ++i)
  {
    lb_.segment(cur_idx, q_lengths_[i]) = robot_models_[names_[i]]->getLowerBound();
    ub_.segment(cur_idx, q_lengths_[i]) = robot_models_[names_[i]]->getUpperBound();
    cur_idx += q_lengths_[i];
  }
  // std::cout << "lb: " << lb_.transpose() << "\nub: " << ub_.transpose() << std::endl;
}

void DualChainConstraintsFunctions::setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat){
  chain_transform_ = vectorsToIsometry(pos,quat);
}

void DualChainConstraintsFunctions::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
  auto model1 = robot_models_.at(names_[0]);
  auto model2 = robot_models_.at(names_[1]);
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
DualChainConstraintsFunctions6D::DualChainConstraintsFunctions6D(const unsigned int ambientDim, const unsigned int coDim) : 
  KinematicsConstraintsFunctions(ambientDim, coDim)
{
  m_ = 6;
}

void DualChainConstraintsFunctions6D::setNames(const std::string & name1, const std::string & name2)
{
  names_[0] = name1;
  names_[1] = name2;

  int n = 0;
  for (int i=0; i<2; ++i)
  {
    q_lengths_[i] = robot_models_[names_[i]]->getNumJoints();
    n += q_lengths_[i];
  }
  std::cout << names_[0] << " and " << names_[1] << std::endl
  << "q len: " << q_lengths_[0] << " and " << q_lengths_[1] << std::endl;

  assert(n == n_);

  lb_.resize(n_);
  ub_.resize(n_);
  int cur_idx = 0;
  for (int i=0; i<2; ++i)
  {
    lb_.segment(cur_idx, q_lengths_[i]) = robot_models_[names_[i]]->getLowerBound();
    ub_.segment(cur_idx, q_lengths_[i]) = robot_models_[names_[i]]->getUpperBound();
    cur_idx += q_lengths_[i];
  }
  std::cout << "lb: " << lb_.transpose() << "\nub: " << ub_.transpose() << std::endl;
}

void DualChainConstraintsFunctions6D::setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat){
  chain_transform_ = vectorsToIsometry(pos,quat);
}

void DualChainConstraintsFunctions6D::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out) const 
{
  auto model1 = robot_models_.at(names_[0]);
  auto model2 = robot_models_.at(names_[1]);
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

DualChainConstraintIK::DualChainConstraintIK(const unsigned int ambientDim, const unsigned int coDim) : 
  DualChainConstraintsFunctions6D(ambientDim, coDim)
{
  target_pose_.setIdentity();
  m_ = 12;
}

void DualChainConstraintIK::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto model1 = robot_models_.at(names_[0]);
  auto model2 = robot_models_.at(names_[1]);
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

  Eigen::Isometry3d pose_error = target_pose_.inverse() * t1;
  Eigen::Matrix3d llg2 = pose_error.linear().log();
  out[6] = pose_error.translation()(0);
  out[7] = pose_error.translation()(1);
  out[8] = pose_error.translation()(2);
  out[9] = llg2(2,1);
  out[10] = llg2(0,2);
  out[11] = llg2(1,0);
}

void DualChainConstraintIK::setTargetPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  target_pose_ = vectorsToIsometry(pos,quat);
}





// class DualChainWithFixedOrientationConstraint : public KinematicsConstraintsFunctions
// {
// public:
//   DualChainWithFixedOrientationConstraint();
//   void function(const Eigen::Ref<const Eigen::VectorXd> &x,
//                                     Eigen::Ref<Eigen::VectorXd> out) override;

//   void setChain(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
//   void setNames(const std::string & name1, const std::string & name2);
//   void setRotErrorRatio(double ratio);

// protected:
//   Eigen::Isometry3d chain_transform_;
//   std::array<std::string, 2> names_;
//   std::array<int,2> q_lengths_;
//   double rot_error_ratio_ {1.0};
  
//   Eigen::Vector3d orientaition_vector_;
//   Eigen::Matrix3d orientation_offset_;
//   std::string name_;
//   int q_length_;
//   int axis_;

// };


DualChainWithFixedOrientationConstraint::DualChainWithFixedOrientationConstraint(const unsigned int ambientDim, const unsigned int coDim) : 
  DualChainConstraintsFunctions6D(ambientDim, coDim)
{
  chain_transform_.setIdentity();
  m_ = 8;
}

void DualChainWithFixedOrientationConstraint::setOrientationVector(const Eigen::Ref<const Eigen::Vector3d> &orientation_vector)
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

void DualChainWithFixedOrientationConstraint::setOrientationOffset(const Eigen::Ref<const Eigen::Matrix3d> &orientation_offset)
{
  orientation_offset_ = orientation_offset;
}


void DualChainWithFixedOrientationConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
  auto model1 = robot_models_.at(names_[0]);
  auto model2 = robot_models_.at(names_[1]);
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
  
  Eigen::Matrix3d R = orientation_offset_.transpose() * t1.linear();
  switch (axis_)
  {
  case 0:
    out[6] = asin(R(1, 0));
    out[7] = asin(R(2, 0));
    break;
  case 1:
    out[6] = asin(R(0, 1));
    out[7] = asin(R(2, 1));
    break;
  case 2:
    out[6] = asin(R(0, 2));
    out[7] = asin(R(1, 2));
    break;
  default:
    break;
  }
  
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
