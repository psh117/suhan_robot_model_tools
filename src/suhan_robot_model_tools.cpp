#include "suhan_robot_model_tools.h"

TRACIKAdapter & KinematicsConstraintsFunctions::addTRACIKAdapter(const std::string & name, const std::string & base_link, const std::string & tip_link, const std::string& URDF_param)
{
  robot_models_[name] = std::make_shared<TRACIKAdapter>(base_link, tip_link, URDF_param);
  return *robot_models_[name];
}

TRACIKAdapter & KinematicsConstraintsFunctions::getTRACIKAdapter(const std::string & name)
{
  return *robot_models_[name];
}

void KinematicsConstraintsFunctions::setTolerance(double tolerance)
{
  tolerance_ = tolerance;
}

void KinematicsConstraintsFunctions::setMaxIterations(int maxIterations)
{
  maxIterations_ = maxIterations;
}

void KinematicsConstraintsFunctions::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) 
{
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::VectorXd t1(1);
    Eigen::VectorXd t2(1);

    // Use a 7-point central difference stencil on each column.
    for (std::size_t j = 0; j < n_; j++)
    {
        const double ax = std::fabs(x[j]);
        // Make step size as small as possible while still giving usable accuracy.
        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (ax >= 1 ? ax : 1);

        // Can't assume y1[j]-y2[j] == 2*h because of precision errors.
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m2 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m3 = (t1 - t2) / (y1[j] - y2[j]);

        out.col(j) = 1.5 * m1 - 0.6 * m2 + 0.1 * m3;

        // Reset for next iteration.
        y1[j] = y2[j] = x[j];
    }
}

bool KinematicsConstraintsFunctions::project(Eigen::Ref<Eigen::VectorXd> x)
{
    // Newton's method
    unsigned int iter = 0;
    double norm = 0;
    Eigen::VectorXd f(1);
    Eigen::MatrixXd j(1, n_);

    const double squaredTolerance = tolerance_ * tolerance_;

    function(x, f);
    while ((norm = f.squaredNorm()) > squaredTolerance && iter++ < maxIterations_)
    {
        jacobian(x, j);
        x -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
        function(x, f);
    }

    return norm < squaredTolerance;
}

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
  
  out[0] = err_p + err_r;
}
