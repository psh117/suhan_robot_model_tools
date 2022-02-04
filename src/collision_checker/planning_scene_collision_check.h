#pragma once

#include <mutex>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
#include <fstream>

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#define DEBUG_FILE(text) \
if(debug_file_.is_open()) \
{ \
  debug_file_ << text << std::endl;\
}

class PlanningSceneCollisionCheck
{
public:
  PlanningSceneCollisionCheck();
  void setArmNames(const std::vector<std::string> &arm_name);
  bool isValid(const Eigen::Ref<const Eigen::VectorXd> &q) const;
  double clearance(const Eigen::Ref<const Eigen::VectorXd> &q) const;

  void updateJoints(const Eigen::Ref<const Eigen::VectorXd> &q);
  geometry_msgs::Pose convertEigenToPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);
  void addMeshFromFile(const std::string & file_name, const std::string &id, 
                       const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addMeshFromFile(const std::string & file_name, geometry_msgs::Pose pose, const std::string &id);
  void updateObjectPose(geometry_msgs::Pose pose, const std::string &id);
  Eigen::Isometry3d getObjectPose(const std::string &id) const;
  void updateObjectPose(const std::string &id, const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, geometry_msgs::Pose pose, const std::string &id);

  void addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, geometry_msgs::Pose pose, const std::string &id);

  void addSphere(const double &dim, const std::string &id,
              const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat);

  void addSphere(const double &dim, geometry_msgs::Pose pose, const std::string &id);

  void addAndAttachObject(const std::string &object_id, const std::string &header_frame, const std::vector<std::string> &touch_links);
  
  void attachObject(const std::string &object_id, const std::string &link_name, const std::vector<std::string> &touch_links);
  void detachObject(const std::string &object_id, const std::string &link_name);
  void detachAllObject(const std::string & link_name);
  void deleteObject(const std::string & object_id);
  std::vector<std::string> getAttachedObjects(const std::string & link_name);
  std::vector<std::string> getAllAttachedObjects();
  void changeCollision(const std::string &name1, const std::string &name2, bool allowed);
  void changeCollisions(const std::string &name1, const std::vector< std::string > &other_names, bool allowed);
  void changeCollisionsAll(const std::string &name1, bool allowed);
  Eigen::Isometry3d geometry_pose_to_isometry(geometry_msgs::Pose geometry_pose);

  bool getArmInCollision(const std::string & arm_name, std::string & collision_arm);
  std::string getCollidingArm(const std::vector<std::string> & arm_names);

  moveit_msgs::PlanningScene getPlanningSceneMsg();
  void publishPlanningSceneMsg();

  void printCurrentCollisionInfos();
  std::stringstream streamCurrentCollisionInfos();

  void openDebugFile() { debug_file_.open( debug_file_prefix_ + "planning_scene_debug.txt"); }
  void setDebugFilePrefix(const std::string &name) { debug_file_prefix_ = name; }

  planning_scene::PlanningScenePtr& getPlanningScene();

private:
  std::vector<std::string> arm_name_;
  // std::array<std::string, 2> move_group_names_ {"panda_left", "panda_right"};
  robot_model::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  ros::NodeHandle nh_;
  ros::Publisher scene_pub_;
  std::string debug_file_prefix_;
  std::ofstream debug_file_;

  mutable std::mutex planning_scene_mtx_;
  mutable collision_detection::CollisionResult last_collision_result_;
  // collision_detection::AllowedCollisionMatrixPtr;
};