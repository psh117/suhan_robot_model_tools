#include "constraints/kinematics_constraint_functions.h"

TRACIKAdapter & KinematicsConstraintsFunctions::addTRACIKAdapter(const std::string & name, const std::string & base_link, const std::string & tip_link, double max_time, double precision, const std::string& URDF_param)
{
  robot_models_[name] = std::make_shared<TRACIKAdapter>(base_link, tip_link, max_time, precision, URDF_param);
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

void KinematicsConstraintsFunctions::setNumFiniteDiff(int num_finite_diff)
{
  num_finite_diff_ = num_finite_diff;
}

void KinematicsConstraintsFunctions::setStepSize(double step_size)
{
  step_size_ = step_size;
}

void KinematicsConstraintsFunctions::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) 
{
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::VectorXd t1(m_);
    Eigen::VectorXd t2(m_);

    // Use a 7-point central difference stencil on each column.
    for (std::size_t j = 0; j < n_; j++)
    {
        const double ax = std::fabs(x[j]);
        // Make step size as small as possible while still giving usable accuracy.
        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (ax >= 1 ? ax : 1);
        if (num_finite_diff_ == 7)
        {
          // Can't assume y1[j]-y2[j] == 2*h because of precision errors.
          y1[j] += h;
          y2[j] -= h;
          function(y1, t1);
          function(y2, t2);
          const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]); // 2h -> 2* 2/3 -> 1.5
          y1[j] += h;
          y2[j] -= h;
          function(y1, t1);
          function(y2, t2);
          const Eigen::VectorXd m2 = (t1 - t2) / (y1[j] - y2[j]); // 4h -> 4 * 3/20 -> 12/20 -> 0.6 
          y1[j] += h;
          y2[j] -= h;
          function(y1, t1);
          function(y2, t2);
          const Eigen::VectorXd m3 = (t1 - t2) / (y1[j] - y2[j]); // 6h -> 6 * 1/60 -> 0.1

          out.col(j) = 1.5 * m1 - 0.6 * m2 + 0.1 * m3;
        }
        else if (num_finite_diff_ == 5)
        {
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

          out.col(j) =  (4 * m1 - m2)/3.0;
        }
        else if (num_finite_diff_ == 3)
        {
          y1[j] += h;
          y2[j] -= h;
          function(y1, t1);
          function(y2, t2);
          const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]);

          out.col(j) = m1;
        }

        // Reset for next iteration.
        y1[j] = y2[j] = x[j];
    }
}

bool KinematicsConstraintsFunctions::project(Eigen::Ref<Eigen::VectorXd> x)
{
    // Newton's method
    unsigned int iter = 0;
    double norm = 0;
    Eigen::VectorXd f(m_);
    Eigen::MatrixXd j(m_, n_);

    const double squaredTolerance = tolerance_ * tolerance_;

    function(x, f);
    while ((norm = f.squaredNorm()) > squaredTolerance && iter++ < maxIterations_)
    {
        jacobian(x, j);
        // Eigen::VectorXd dx = j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
        // double max_dist = std::max(dx.maxCoeff(),-dx.minCoeff());
        // if (max_dist > step_size_)
        // {
        //   dx *= step_size_/max_dist;
        // }
        // x -= dx; //step_size_ * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
        x -= step_size_ * j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
        function(x, f);
        
        for (int i=0; i<x.size(); i++)
        {
          if (x[i] < lb_[i])
            return false;
            
          if (x[i] > ub_[i])
            return false;
        }
    }

    return norm < squaredTolerance;
}

void KinematicsConstraintsFunctions::setDebugLevel(int debug_level)
{
  debug_level_ = debug_level;
}