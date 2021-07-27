#include "trac_ik_adapter.h"

Eigen::Matrix3d getEigenRotation(const KDL::Rotation & r)
{
  Eigen::Matrix3d matrix;
  for (int i=0 ;i<9; i++)
  {
     matrix(i/3, i%3) = r.data[i];
  }
  return matrix;
}

Eigen::Vector3d getEigenVector(const KDL::Vector& v)
{
  Eigen::Vector3d vector;
  for (int i=0; i<3; i++)
  {
    vector(i) = v(i);
  }
  return vector;
}

Eigen::Isometry3d getEigenFrame(const KDL::Frame & frame)
{
  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform.translation() = getEigenVector(frame.p);
  transform.linear() = getEigenRotation(frame.M);
  return transform;
}

KDL::Rotation getKDLRotation(const Eigen::Matrix3d & matrix)
{
  KDL::Rotation r;
  for (int i=0 ;i<9; i++)
  {
    r.data[i] = matrix(i/3, i%3);
  }
  return r;
}

KDL::Vector getKDLVector(const Eigen::Vector3d& vector)
{
  KDL::Vector v;
  for (int i=0; i<3; i++)
  {
    v(i) = vector(i);
  }
  return v;
}

KDL::Frame getKDLFrame(const Eigen::Isometry3d & transform)
{
  KDL::Frame frame;
  frame.p = getKDLVector(transform.translation());
  frame.M = getKDLRotation(transform.linear());
  return frame;
}

TRACIKAdapter::TRACIKAdapter(const std::string& base_link, 
                             const std::string& tip_link, 
                             double max_time, double precision,
                             const std::string& URDF_param)
 : trac_ik_solver_(base_link, tip_link, URDF_param, max_time, precision) 
                                  // 0.2, 1e-4 TRAC_IK::Speed
{ 
  std::scoped_lock _lock(iK_solver_mutex_);
  last_transform_.setIdentity(); 
  bool valid = trac_ik_solver_.getKDLChain(chain_);
  if (!valid){ROS_ERROR("There was no valid KDL chain found");return;}

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));

  KDL::JntArray ll, ul; //lower joint limits, upper joint limits
  valid = trac_ik_solver_.getKDLLimits(ll, ul);
  if (!valid){ROS_ERROR("There were no valid KDL joint limits found");return;}

  n_joint_ = chain_.getNrOfJoints();
  assert(n_joint_ == ll.data.size());
  assert(n_joint_ == ul.data.size());

  lb_ = ll.data;
  ub_ = ul.data;
  // Create Nominal chain configuration midway between all joint limits
  nominal_ = KDL::JntArray(n_joint_);
  for (uint j = 0; j < nominal_.data.size(); j++)
  {
      nominal_(j) = (ll(j) + ul(j)) / 2.0;
  }
}

bool TRACIKAdapter::solve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution)
{
  return solve(nominal_.data, transform, solution);
}

Eigen::VectorXd TRACIKAdapter::getRandomConfig()
{
  Eigen::VectorXd length = (ub_ - lb_) / 2. ;
  // std::cout << ub_.transpose() << std::endl;
  // std::cout << lb_.transpose() << std::endl;
  // std::cout << length.transpose() << std::endl;
  // std::cout << length.asDiagonal().toDenseMatrix() << std::endl;
  return length.asDiagonal() * Eigen::VectorXd::Random(n_joint_) + length + lb_;
}

Eigen::VectorXd TRACIKAdapter::getGaussianRandomConfig(const Eigen::Ref<const Eigen::VectorXd> &q0, double sigma)
{
  Eigen::VectorXd output(lb_.size());
  std::normal_distribution<double> distribution(0.0, sigma);
  for(int i=0; i<lb_.size(); i++)
  {
    output(i) = q0(i) + distribution(rg_);
    if (output(i) > ub_(i)) output(i) = ub_(i);
    if (output(i) < lb_(i)) output(i) = lb_(i);
  }
  return output;
}

Eigen::VectorXd TRACIKAdapter::getGaussianRandomConfig(double sigma)
{
  Eigen::VectorXd center = (ub_ + lb_) / 2. ;
  
  Eigen::VectorXd output(lb_.size());
  std::normal_distribution<double> distribution(0.0, sigma);
  for(int i=0; i<lb_.size(); i++)
  {
    output(i) = center(i) + distribution(rg_);
    if (output(i) > ub_(i)) output(i) = ub_(i);
    if (output(i) < lb_(i)) output(i) = lb_(i);
  }
  return output;
}

// be careful not to be over the available area
Eigen::VectorXd TRACIKAdapter::getStrictRandomConfig(double margin)
{
  Eigen::VectorXd margin_vector;
  margin_vector.setConstant(ub_.size(), margin);
  Eigen::VectorXd length = (ub_ - lb_) / 2.; // 1-directional length
  return (length-margin_vector).asDiagonal() * Eigen::VectorXd::Random(n_joint_) + (length + lb_); // length * (-1~1) + center
}

bool TRACIKAdapter::randomSolveWithStartOnce(const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution)
{
  if (transform.isApprox(last_transform_))
  {
    return randomSolve(transform, solution);
  }
  else
  {
    last_transform_ = transform;
    return solve(q0, transform, solution);
  }
}

bool TRACIKAdapter::randomSolve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution)
{
  return solve(getRandomConfig(), transform, solution);
}

bool TRACIKAdapter::randomGaussianSolve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution, double sigma)
{
  return solve(getGaussianRandomConfig(sigma), transform, solution);
}

bool TRACIKAdapter::randomStrictSolve(const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution, double margin)
{
  return solve(getStrictRandomConfig(margin), transform, solution);
}

void TRACIKAdapter::setBounds(const Eigen::Ref<const Eigen::VectorXd> &lb, const Eigen::Ref<const Eigen::VectorXd> &ub)
{
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits
  bool valid = trac_ik_solver_.getKDLLimits(ll, ul);
  ll.data = lb;
  ul.data = ub;
  valid = trac_ik_solver_.setKDLLimits(ll,ul);
  if (!valid)
  {
    ROS_ERROR("setting bounds fails");
  }
}

void TRACIKAdapter::setToleranceBounds(double px, double py, double pz, double ox, double oy, double oz)
{
  bounds_.vel(0) = px;
  bounds_.vel(1) = py;
  bounds_.vel(2) = pz;
  bounds_.rot(0) = ox;
  bounds_.rot(1) = oy;
  bounds_.rot(2) = oz;
}

bool TRACIKAdapter::isValid(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  for (int i=0; i<lb_.size(); i++)
  {
    if (q(i) < lb_(i)) return false;
    if (q(i) > ub_(i)) return false;
  }
  return true;
}

bool TRACIKAdapter::solve(const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution)
{
  std::scoped_lock _lock(iK_solver_mutex_);

  KDL::Frame target_frame;
  KDL::JntArray jarr_q0;
  KDL::JntArray result_q;
  result_q.resize(n_joint_);
  jarr_q0.data = q0;

  target_frame = getKDLFrame(transform);
  KDL::Twist bounds;
  // 1e-1
  // bounds.rot(0) = 1e-1;
  // bounds.rot(1) = 1e-1;
  // bounds.rot(2) = 1e-1;
  if (trac_ik_solver_.CartToJnt(jarr_q0, target_frame, result_q, bounds_) >= 0)
  {
    solution = result_q.data;
    return true;
  }
  solution = result_q.data;
  return false;
}

Eigen::Isometry3d TRACIKAdapter::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  std::scoped_lock _lock(iK_solver_mutex_);
  KDL::JntArray jarr_q;
  KDL::Frame frame;
  
  jarr_q.data = q;
  fk_solver_->JntToCart(jarr_q, frame);

  return getEigenFrame(frame);
}