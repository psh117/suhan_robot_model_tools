
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/suite/indexing/map_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include "eigen_tools/eigen_tools.h"
#include "collision_checker/planning_scene_collision_check.h"

#include "constraints/kinematics_constraint_functions.h"
#include "constraints/orientation_constraint_functions.h"
#include "constraints/multi_chain_constraint_functions.h"
#include "constraints/dual_chain_constraint_functions.h"

#include "visual_sim/visual_sim.h"

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

  bp::class_<std::vector<int> > ("IntVector")
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
      .def("get_jacobian_matrix", &TRACIKAdapter::getJacobianMatrix)
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
      .def("set_step_size", &DualChainConstraintsFunctions::setStepSize)
      .def("set_debug_level", &DualChainConstraintsFunctions::setDebugLevel)
      .def("set_early_stopping", &DualChainConstraintsFunctions::setEarlyStopping)
      ;
  bp::class_<DualChainConstraintsFunctions6D, boost::noncopyable>("DualChainConstraintsFunctions6D")
      .def("add_trac_ik_adapter", &DualChainConstraintsFunctions6D::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &DualChainConstraintsFunctions6D::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &DualChainConstraintsFunctions6D::project)
      .def("jacobian", &DualChainConstraintsFunctions6D::jacobian)
      .def("function", &DualChainConstraintsFunctions6D::function)
      .def("set_chain", &DualChainConstraintsFunctions6D::setChain)
      .def("set_names", &DualChainConstraintsFunctions6D::setNames)
      .def("set_tolerance", &DualChainConstraintsFunctions6D::setTolerance)
      .def("set_max_iterations", &DualChainConstraintsFunctions6D::setMaxIterations)
      .def("set_num_finite_diff", &DualChainConstraintsFunctions6D::setNumFiniteDiff)
      .def("set_rot_error_ratio", &DualChainConstraintsFunctions6D::setRotErrorRatio)
      .def("set_step_size", &DualChainConstraintsFunctions6D::setStepSize)
      .def("set_debug_level", &DualChainConstraintsFunctions6D::setDebugLevel)
      .def("set_early_stopping", &DualChainConstraintsFunctions6D::setEarlyStopping)
      ;
  bp::class_<DualChainConstraintIK, boost::noncopyable>("DualChainConstraintIK")
      .def("add_trac_ik_adapter", &DualChainConstraintIK::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &DualChainConstraintIK::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &DualChainConstraintIK::project)
      .def("jacobian", &DualChainConstraintIK::jacobian)
      .def("function", &DualChainConstraintIK::function)
      .def("set_chain", &DualChainConstraintIK::setChain)
      .def("set_names", &DualChainConstraintIK::setNames)
      .def("set_tolerance", &DualChainConstraintIK::setTolerance)
      .def("set_max_iterations", &DualChainConstraintIK::setMaxIterations)
      .def("set_num_finite_diff", &DualChainConstraintIK::setNumFiniteDiff)
      .def("set_rot_error_ratio", &DualChainConstraintIK::setRotErrorRatio)
      .def("set_step_size", &DualChainConstraintIK::setStepSize)
      .def("set_target_pose", &DualChainConstraintIK::setTargetPose)
      .def("set_debug_level", &DualChainConstraintIK::setDebugLevel)
      .def("set_early_stopping", &DualChainConstraintIK::setEarlyStopping)
      ;
  
  bp::class_<OrientationConstraintFunctions, boost::noncopyable>("OrientationConstraintFunctions")
      .def("add_trac_ik_adapter", &OrientationConstraintFunctions::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &OrientationConstraintFunctions::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &OrientationConstraintFunctions::project)
      .def("jacobian", &OrientationConstraintFunctions::jacobian)
      .def("function", &OrientationConstraintFunctions::function)
      .def("set_name", &OrientationConstraintFunctions::setName)
      .def("set_orientation_vector", &OrientationConstraintFunctions::setOrientationVector)
      .def("set_orientation_offset", &OrientationConstraintFunctions::setOrientationOffset)
      .def("set_tolerance", &OrientationConstraintFunctions::setTolerance)
      .def("set_max_iterations", &OrientationConstraintFunctions::setMaxIterations)
      .def("set_num_finite_diff", &OrientationConstraintFunctions::setNumFiniteDiff)
      .def("set_step_size", &OrientationConstraintFunctions::setStepSize)
      .def("set_debug_level", &OrientationConstraintFunctions::setDebugLevel)
      .def("set_early_stopping", &OrientationConstraintFunctions::setEarlyStopping)
      ;
      
  bp::class_<OrientationConstrainedIK, boost::noncopyable>("OrientationConstrainedIK")
      .def("add_trac_ik_adapter", &OrientationConstrainedIK::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &OrientationConstrainedIK::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &OrientationConstrainedIK::project)
      .def("jacobian", &OrientationConstrainedIK::jacobian)
      .def("function", &OrientationConstrainedIK::function)
      .def("set_name", &OrientationConstrainedIK::setName)
      .def("set_orientation_vector", &OrientationConstrainedIK::setOrientationVector)
      .def("set_orientation_offset", &OrientationConstrainedIK::setOrientationOffset)
      .def("set_target_position", &OrientationConstrainedIK::setTargetPosition)
      .def("set_tolerance", &OrientationConstrainedIK::setTolerance)
      .def("set_max_iterations", &OrientationConstrainedIK::setMaxIterations)
      .def("set_num_finite_diff", &OrientationConstrainedIK::setNumFiniteDiff)
      .def("set_step_size", &OrientationConstrainedIK::setStepSize)
      .def("set_debug_level", &OrientationConstrainedIK::setDebugLevel)
      .def("set_early_stopping", &OrientationConstrainedIK::setEarlyStopping)
      ;
  
  bp::class_<MultiChainConstraintFunctions, boost::noncopyable>("MultiChainConstraintFunctions")
      .def("add_trac_ik_adapter", &MultiChainConstraintFunctions::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &MultiChainConstraintFunctions::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &MultiChainConstraintFunctions::project)
      .def("jacobian", &MultiChainConstraintFunctions::jacobian)
      .def("function", &MultiChainConstraintFunctions::function)
      .def("set_chains", &MultiChainConstraintFunctions::setChains)
      .def("set_chains_from_joints", &MultiChainConstraintFunctions::setChainsFromJoints)
      .def("set_names", &MultiChainConstraintFunctions::setNames)
      .def("set_tolerance", &MultiChainConstraintFunctions::setTolerance)
      .def("set_max_iterations", &MultiChainConstraintFunctions::setMaxIterations)
      .def("set_num_finite_diff", &MultiChainConstraintFunctions::setNumFiniteDiff)
      .def("set_rot_error_ratio", &MultiChainConstraintFunctions::setRotErrorRatio)
      .def("set_step_size", &MultiChainConstraintFunctions::setStepSize)
      .def("set_debug_level", &MultiChainConstraintFunctions::setDebugLevel)
      .def("set_early_stopping", &MultiChainConstraintFunctions::setEarlyStopping)
      ;

  bp::class_<MultiChainConstraintIK, boost::noncopyable>("MultiChainConstraintIK")
      .def("add_trac_ik_adapter", &MultiChainConstraintIK::addTRACIKAdapter, bp::return_internal_reference<>())
      .def("get_trac_ik_adapter", &MultiChainConstraintIK::getTRACIKAdapter, bp::return_internal_reference<>())
      .def("project", &MultiChainConstraintIK::project)
      .def("jacobian", &MultiChainConstraintIK::jacobian)
      .def("function", &MultiChainConstraintIK::function)
      .def("set_chains", &MultiChainConstraintIK::setChains)
      .def("set_chains_from_joints", &MultiChainConstraintIK::setChainsFromJoints)
      .def("set_names", &MultiChainConstraintIK::setNames)
      .def("set_tolerance", &MultiChainConstraintIK::setTolerance)
      .def("set_max_iterations", &MultiChainConstraintIK::setMaxIterations)
      .def("set_num_finite_diff", &MultiChainConstraintIK::setNumFiniteDiff)
      .def("set_rot_error_ratio", &MultiChainConstraintIK::setRotErrorRatio)
      .def("set_step_size", &MultiChainConstraintIK::setStepSize)
      .def("set_target_pose", &MultiChainConstraintIK::setTargetPose)
      .def("set_debug_level", &MultiChainConstraintIK::setDebugLevel)
      .def("set_early_stopping", &MultiChainConstraintIK::setEarlyStopping)
      ;
//   bp::class_<TripleChainConstraintsFunctions, boost::noncopyable>("TripleChainConstraintsFunctions")
//       .def("add_trac_ik_adapter", &TripleChainConstraintsFunctions::addTRACIKAdapter, bp::return_internal_reference<>())
//       .def("get_trac_ik_adapter", &TripleChainConstraintsFunctions::getTRACIKAdapter, bp::return_internal_reference<>())
//       .def("project", &TripleChainConstraintsFunctions::project)
//       .def("jacobian", &TripleChainConstraintsFunctions::jacobian)
//       .def("function", &TripleChainConstraintsFunctions::function)
//       .def("set_chain", &TripleChainConstraintsFunctions::setChain)
//       .def("set_names", &TripleChainConstraintsFunctions::setNames)
//       .def("set_tolerance", &TripleChainConstraintsFunctions::setTolerance)
//       .def("set_max_iterations", &TripleChainConstraintsFunctions::setMaxIterations)
//       .def("set_num_finite_diff", &TripleChainConstraintsFunctions::setNumFiniteDiff)
//       .def("set_rot_error_ratio", &TripleChainConstraintsFunctions::setRotErrorRatio)
//       ;

  bool (ompl::base::Constraint::*project1)(Eigen::Ref<Eigen::VectorXd> x) const = &ompl::base::Constraint::project;
  void (PlanningSceneCollisionCheck::*addBox1)(const Eigen::Ref<const Eigen::Vector3d> &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat) = &PlanningSceneCollisionCheck::addBox;
  void (PlanningSceneCollisionCheck::*addCylinder1)(const Eigen::Ref<const Eigen::Vector2d> &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat) = &PlanningSceneCollisionCheck::addCylinder;
  void (PlanningSceneCollisionCheck::*addSphere)(const double &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat) = &PlanningSceneCollisionCheck::addSphere;
  void (PlanningSceneCollisionCheck::*addMeshFromFile)(const std::string & file_name, const std::string &id, 
                       const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat) = &PlanningSceneCollisionCheck::addMeshFromFile;
  void (PlanningSceneCollisionCheck::*updateObjectPose1)(const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat) = &PlanningSceneCollisionCheck::updateObjectPose;

  bp::class_<PlanningSceneCollisionCheck, boost::noncopyable>("PlanningSceneCollisionCheck", bp::init<const std::string&>())
      .def("set_group_names_and_dofs", &PlanningSceneCollisionCheck::setGroupNamesAndDofs)
      .def("is_valid", &PlanningSceneCollisionCheck::isValid)
      .def("attach_object", &PlanningSceneCollisionCheck::attachObject)
      .def("detach_object", &PlanningSceneCollisionCheck::detachObject)
      .def("publish_planning_scene_msg", &PlanningSceneCollisionCheck::publishPlanningSceneMsg)
      .def("print_current_collision_infos", &PlanningSceneCollisionCheck::printCurrentCollisionInfos)
      .def("update_object_pose", updateObjectPose1)
      .def("update_joints", &PlanningSceneCollisionCheck::updateJoints)
      .def("add_box", addBox1)
      .def("detach_all_objects", &PlanningSceneCollisionCheck::detachAllObjects)
      .def("add_cylinder", addCylinder1)
      .def("add_sphere", addSphere)
      .def("add_mesh_from_file", addMeshFromFile)
      .def("set_frame_id", &PlanningSceneCollisionCheck::setFrameID)
    //   .def("get_planning_scene", &PlanningSceneCollisionCheck::getPlanningScene)
      .def("get_planning_scene", &PlanningSceneCollisionCheck::getPlanningScene, bp::return_internal_reference<>())
      ;

  bp::class_<std::shared_ptr<planning_scene::PlanningScene>, boost::noncopyable>("PlanningScene", bp::no_init);

  bp::class_<VisualSim, boost::noncopyable>("VisualSim")
      .def("lookat", &VisualSim::lookat)
      .def("set_cam_pose", &VisualSim::setCamPose)
      .def("set_cam_pos", &VisualSim::setCamPos)
      .def("load_scene", &VisualSim::loadScene)
      .def("generate_depth_image", &VisualSim::generateDepthImage)
      .def("generate_voxel_occupancy", &VisualSim::generateVoxelOccupancy)
      .def("set_grid_resolution", &VisualSim::setGridResolution)
      .def("set_scene_bounds", &VisualSim::setSceneBounds)
      ;

}