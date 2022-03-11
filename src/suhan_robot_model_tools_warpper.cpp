
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include "suhan_robot_model_tools.h"
#include "eigen_tools/eigen_tools.h"
#include "collision_checker/planning_scene_collision_check.h"

BOOST_PYTHON_MODULE(suhan_robot_model_tools_wrapper_cpp)
{
  namespace ob = ompl::base;
  namespace bp = boost::python;
  eigenpy::enableEigenPy();

  bp::class_<std::pair<Eigen::Vector3d, Eigen::Vector4d> >("IntPair")
      .def_readwrite("first", &std::pair<Eigen::Vector3d, Eigen::Vector4d>::first)
      .def_readwrite("second", &std::pair<Eigen::Vector3d, Eigen::Vector4d>::second);
      
  bp::def("vectors_to_isometry",vectorsToIsometry);
  bp::def("isometry_to_vectors",isometryToVectors);

  bp::class_<std::vector<std::string> > ("NameVector")
    .def(boost::python::vector_indexing_suite<std::vector<std::string> >())
  ;

  bp::class_<std::vector<std::string> > ("IntVector")
    .def(boost::python::vector_indexing_suite<std::vector<int> >())
  ;
  bp::class_<Eigen::Isometry3d>("Isometry3d");

  bool (TRACIKAdapter::*solve1)
    (const Eigen::Ref<const Eigen::VectorXd> &q0, const Eigen::Isometry3d & transform, Eigen::Ref<Eigen::VectorXd> solution) 
    = &TRACIKAdapter::solve;

  bp::class_<TRACIKAdapter, boost::noncopyable>("TRACIKAdapter", bp::init<const std::string& , const std::string&, double, double, const std::string&>())
      .def("get_lower_bound", &TRACIKAdapter::getLowerBound)
      .def("get_upper_bound", &TRACIKAdapter::getUpperBound)
      .def("get_num_joints", &TRACIKAdapter::getNumJoints)
      .def("is_valid", &TRACIKAdapter::isValid)
      .def("solve", solve1)
      .def("forward_kinematics", &TRACIKAdapter::forwardKinematics)
      .def("set_bounds", &TRACIKAdapter::setBounds)
      .def("set_tolerance_bounds", &TRACIKAdapter::setToleranceBounds)
      ;

  bp::class_<DualChainConstraintsFunctions, boost::noncopyable>("DualChainConstraintsFunctions")
      .def("add_trac_ik_adapter", &DualChainConstraintsFunctions::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &DualChainConstraintsFunctions::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &DualChainConstraintsFunctions::project)
      .def("jacobian", &DualChainConstraintsFunctions::jacobian)
      .def("function", &DualChainConstraintsFunctions::function)
      .def("set_chain", &DualChainConstraintsFunctions::setChain)
      .def("set_names", &DualChainConstraintsFunctions::setNames)
      .def("set_tolerance", &DualChainConstraintsFunctions::setTolerance)
      .def("set_max_iterations", &DualChainConstraintsFunctions::setMaxIterations)
      .def("set_num_finite_diff", &DualChainConstraintsFunctions::setNumFiniteDiff)
      .def("set_rot_error_ratio", &DualChainConstraintsFunctions::setRotErrorRatio)
      ;
      
  bp::class_<TripleChainConstraintsFunctions, boost::noncopyable>("TripleChainConstraintsFunctions")
      .def("add_trac_ik_adapter", &TripleChainConstraintsFunctions::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &TripleChainConstraintsFunctions::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &TripleChainConstraintsFunctions::project)
      .def("jacobian", &TripleChainConstraintsFunctions::jacobian)
      .def("function", &TripleChainConstraintsFunctions::function)
      .def("set_chain", &TripleChainConstraintsFunctions::setChain)
      .def("set_names", &TripleChainConstraintsFunctions::setNames)
      .def("set_tolerance", &TripleChainConstraintsFunctions::setTolerance)
      .def("set_max_iterations", &TripleChainConstraintsFunctions::setMaxIterations)
      .def("set_num_finite_diff", &TripleChainConstraintsFunctions::setNumFiniteDiff)
      .def("set_rot_error_ratio", &TripleChainConstraintsFunctions::setRotErrorRatio)
      ;


  bp::class_<PlanningSceneCollisionCheck, boost::noncopyable>("PlanningSceneCollisionCheck")
      .def("set_group_names_and_dofs", &PlanningSceneCollisionCheck::setGroupNamesAndDofs)
      .def("is_valid", &PlanningSceneCollisionCheck::isValid)
      .def("publish_planning_scene_msg", &PlanningSceneCollisionCheck::publishPlanningSceneMsg)
      .def("print_current_collision_infos", &PlanningSceneCollisionCheck::printCurrentCollisionInfos)
      .def("update_joints", &PlanningSceneCollisionCheck::updateJoints)
      ;
}