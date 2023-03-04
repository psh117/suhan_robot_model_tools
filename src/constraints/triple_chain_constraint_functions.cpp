#include "constraints/triple_chain_constraint_functions.h"

void TripleChainConstraintsFunctions::setNames(const std::string & name1, const std::string & name2, const std::string & name3)
{
  names_[0] = name1;
  names_[1] = name2;
  names_[2] = name3;

  n_ = 0;
  for (int i=0; i<3; ++i)
  {
    q_lengths_[i] = robot_models_[names_[i]]->getNumJoints();
    n_ += q_lengths_[i];
  }
  std::cout << names_[0] << " and " << names_[1]  << " and " << names_[2] << std::endl
  << "q len: " << q_lengths_[0] << " and " << q_lengths_[1] << " and " << q_lengths_[2] << std::endl;

}

void TripleChainConstraintsFunctions::setChain(
  const Eigen::Ref<const Eigen::VectorXd> &q1, 
  const Eigen::Ref<const Eigen::VectorXd> &q2, 
  const Eigen::Ref<const Eigen::VectorXd> &q3)
{
  auto & model1 = robot_models_[names_[0]];
  auto & model2 = robot_models_[names_[1]];
  auto & model3 = robot_models_[names_[2]];
  
  auto t1 = model1->forwardKinematics(q1);
  auto t2 = model2->forwardKinematics(q2);
  auto t3 = model3->forwardKinematics(q3);

  chain_transform_[0] = t1.inverse() * t2;
  chain_transform_[1] = t1.inverse() * t3;
  std::cout << cgreen << "[info] chain is successfully registered"  << creset << std::endl
            << "[t1]" << std::endl 
            << t1.matrix() << std::endl
            << "[t2]" << std::endl 
            << t2.matrix() << std::endl
            << "[t3]" << std::endl 
            << t3.matrix() << std::endl
            << "[chain_transform_[0]]" << std::endl 
            << chain_transform_[0].matrix() << std::endl
            << "[chain_transform_[1]]" << std::endl 
            << chain_transform_[1].matrix() << std::endl;
}

void TripleChainConstraintsFunctions::function(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  Eigen::Ref<Eigen::VectorXd> out)
{
  auto & model1 = robot_models_[names_[0]];
  auto & model2 = robot_models_[names_[1]];
  auto & model3 = robot_models_[names_[2]];

  const Eigen::Ref<const Eigen::VectorXd> q1 = x.head(q_lengths_[0]);
  const Eigen::Ref<const Eigen::VectorXd> q2 = x.segment(q_lengths_[0], q_lengths_[1]); 
  const Eigen::Ref<const Eigen::VectorXd> q3 = x.tail(q_lengths_[2]);

  auto t1 = model1->forwardKinematics(q1);
  auto t2 = model2->forwardKinematics(q2);
  auto t3 = model3->forwardKinematics(q3);
  
  Eigen::Isometry3d current_chain_1 = t1.inverse() * t2;
  Eigen::Isometry3d current_chain_2 = t1.inverse() * t3;
  // std::cout << "[deb] current_chain" << std::endl
  //           << "[t1]" << std::endl 
  //           << t1.matrix() << std::endl
  //           << "[t2]" << std::endl 
  //           << t2.matrix() << std::endl
  //           << "[t3]" << std::endl 
  //           << t3.matrix() << std::endl
  //           << "[current_chain_1]" << std::endl 
  //           << current_chain_1.matrix() << std::endl
  //           << "[current_chain_2]" << std::endl 
  //           << current_chain_2.matrix() << std::endl;
  auto d = [this](const Eigen::Isometry3d & current_chain, const Eigen::Isometry3d & chain_transform){
    Eigen::Quaterniond current_quat(current_chain.linear());
    Eigen::Quaterniond init_quat(chain_transform.linear());

    double err_r = current_quat.angularDistance(init_quat);
    double err_p = (current_chain.translation() - chain_transform.translation()).squaredNorm();
    
    return err_r * err_r + err_p * this->rot_error_ratio_;
  };

  out[0] = d(current_chain_1, chain_transform_[0]) + 
           d(current_chain_2, chain_transform_[1]);
}

void TripleChainConstraintsFunctions::setRotErrorRatio(double ratio)
{
  rot_error_ratio_ = ratio;
}