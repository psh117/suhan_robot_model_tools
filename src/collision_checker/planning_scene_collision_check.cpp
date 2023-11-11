#include "collision_checker/planning_scene_collision_check.h"

// TODO: make it load urdf and srdf 
// robot_model_loader::RobotModelLoader robot_model_loader("urdf", "srdf");

PlanningSceneCollisionCheck::PlanningSceneCollisionCheck(const std::string & topic_name)
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();
  planning_scene_ = std::make_shared<planning_scene::PlanningScene> (robot_model_);
  scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>(topic_name, 1);
  // acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(planning_scene_->getAllowedCollisionMatrix());
};

void PlanningSceneCollisionCheck::setGroupNamesAndDofs(const std::vector<std::string> &arm_name, const std::vector<int> & dofs)
{
  assert(arm_name.size() == dofs.size());

  int len_groups = arm_name.size();
  group_infos_.resize(len_groups);

  for (int i=0; i<len_groups; ++i)
  {
    group_infos_[i].first = arm_name[i];
    group_infos_[i].second = dofs[i];
  }

}

bool PlanningSceneCollisionCheck::isValid(const Eigen::Ref<const Eigen::VectorXd> &q) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  robot_state::RobotState current_state = planning_scene_->getCurrentState();
  
  int current_seg_index = 0;
  for (auto & group_info : group_infos_)
  {
    int dof = group_info.second;
    const auto & q_seg = q.segment(current_seg_index, dof);
    const std::vector<double> joint_values(q_seg.data(), q_seg.data() + dof);
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_info.first);
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    current_seg_index += dof;
  }
  // for (int i=0; i<arm_name_.size(); i++)
  // {
  //   const auto & q_seg = q.segment<7>(i*7);
  //   const std::vector<double> joint_values(q_seg.data(), q_seg.data() + 7);
  //   const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(arm_name_[i]);
  //   current_state.setJointGroupPositions(joint_model_group, joint_values);
  // }
  planning_scene_->checkCollision(collision_request, collision_result, current_state);

  last_collision_result_ = collision_result; 
  // DEBUG_FILE("q: " << q.transpose());
  // DEBUG_FILE("validity: " << !collision_result.collision);
  // collision_result->
  // planning_scene_->checkCollision()

  return !collision_result.collision;
}

bool PlanningSceneCollisionCheck::isCurrentValid() const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  robot_state::RobotState current_state = planning_scene_->getCurrentState();
  
  planning_scene_->checkCollision(collision_request, collision_result, current_state);

  last_collision_result_ = collision_result; 
  
  return !collision_result.collision;
}

void PlanningSceneCollisionCheck::setJointGroupPositions(const std::string& name, const Eigen::Ref<const Eigen::VectorXd> &q)
{
  std::scoped_lock _lock(planning_scene_mtx_);
  robot_state::RobotState & current_state = planning_scene_->getCurrentStateNonConst();
  current_state.setJointGroupPositions(name, q);
}

double PlanningSceneCollisionCheck::clearance(const Eigen::Ref<const Eigen::VectorXd> &q) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.distance = true;
  robot_state::RobotState current_state = planning_scene_->getCurrentState();

  int current_seg_index = 0;
  for (auto & group_info : group_infos_)
  {
    int dof = group_info.second;
    const auto & q_seg = q.segment(current_seg_index, dof);
    const std::vector<double> joint_values(q_seg.data(), q_seg.data() + dof);
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(group_info.first);
    current_state.setJointGroupPositions(joint_model_group, joint_values);
    current_seg_index += dof;
  }

  // for (int i=0; i<arm_name_.size(); i++)
  // {
  //   const auto & q_seg = q.segment<7>(i*7);
  //   const std::vector<double> joint_values(q_seg.data(), q_seg.data() + 7);
  //   const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(arm_name_[i]);
  //   current_state.setJointGroupPositions(joint_model_group, joint_values);
  // }
  planning_scene_->checkCollision(collision_request, collision_result, current_state);

  last_collision_result_ = collision_result; 
  
  if (collision_result.collision)
  {
    return 0.0;
  }

  return collision_result.distance;
}

void PlanningSceneCollisionCheck::updateJoints(const Eigen::Ref<const Eigen::VectorXd> &q)
{
  std::scoped_lock _lock(planning_scene_mtx_);
  // std::cout << "void updateJoints(const Eigen::Ref<const Eigen::VectorXd> &q)" << std::endl;
  robot_state::RobotState & current_state = planning_scene_->getCurrentStateNonConst();

  int current_seg_index = 0;
  for (auto & group_info : group_infos_)
  {
    int dof = group_info.second;
    const auto & q_seg = q.segment(current_seg_index, dof);
    current_state.setJointGroupPositions(group_info.first, q_seg);
    current_seg_index += dof;
  }

  // for (int i=0; i<arm_name_.size(); i++)
  // {
  //   Eigen::Matrix<double, 7, 1> q_seg = q.segment<7>(i*7);
  //   // std::vector<double> joint_values(q_seg.data(), q_seg.data() + 7);
  //   // const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(arm_name_[i]);
  //   // current_state.setJointGroupPositions(joint_model_group, joint_values);
  //   // std::cout << arm_name_[i] << ": " << q_seg.transpose() << std::endl;
  //   current_state.setJointGroupPositions(arm_name_[i], q_seg);
  // };
  // planning_scene_->setCurrentState(current_state);
}

geometry_msgs::Pose PlanningSceneCollisionCheck::convertEigenToPose(const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::Pose pose;
  pose.position.x = pos(0);
  pose.position.y = pos(1);
  pose.position.z = pos(2);
  pose.orientation.x = quat(0);
  pose.orientation.y = quat(1);
  pose.orientation.z = quat(2);
  pose.orientation.w = quat(3);
  return pose;
}
void PlanningSceneCollisionCheck::addMeshFromFile(const std::string & file_name, const std::string &id, 
                      const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addMeshFromFile(file_name, pose, id);
}

void PlanningSceneCollisionCheck::addMeshFromFile(const std::string & file_name, geometry_msgs::Pose pose, const std::string &id)
{
  shapes::Mesh *mesh = shapes::createMeshFromResource(file_name);
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);

  shape_msgs::Mesh shape_msgs_mesh = boost::get<shape_msgs::Mesh>(shape_msg);

  moveit_msgs::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  geometry_msgs::Pose empty_pose;
  empty_pose.orientation.w = 1.0;
  co.meshes.push_back(shape_msgs_mesh);
  co.mesh_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::CollisionObject::ADD;

{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}

}

void PlanningSceneCollisionCheck::updateObjectPose(geometry_msgs::Pose pose, const std::string &id)
{
  moveit_msgs::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
#if ROS_VERSION_MINOR <= 14
  co.mesh_poses.push_back(pose);
#else // >= 15
  co.pose = pose;
#endif
  co.operation = moveit_msgs::CollisionObject::MOVE;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}
}

void PlanningSceneCollisionCheck::updateObjectPose(const std::string &id, const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::Pose pose;
  pose = convertEigenToPose(pos, quat);
  updateObjectPose(pose, id);
}

Eigen::Isometry3d PlanningSceneCollisionCheck::getObjectPose(const std::string &id) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  const auto T = planning_scene_->getWorld()->getObject(id)->shape_poses_[0];
  return T;
}

void PlanningSceneCollisionCheck::addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, const std::string &id,
            const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addBox(dim, pose, id);
  // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry('hand', id);
}

void PlanningSceneCollisionCheck::addBox(const Eigen::Ref<const Eigen::Vector3d> &dim, geometry_msgs::Pose pose, const std::string &id)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = dim(0);
  primitive.dimensions[1] = dim(1);
  primitive.dimensions[2] = dim(2);

  geometry_msgs::Pose empty_pose;
  empty_pose.orientation.w = 1.0;

  moveit_msgs::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::CollisionObject::ADD;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}
  // ROS_INFO("ADD BOX!!");
  // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry('hand', id);
}

void PlanningSceneCollisionCheck::addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, const std::string &id,
            const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addCylinder(dim, pose, id);
  // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry('hand', id);
}

void PlanningSceneCollisionCheck::addCylinder(const Eigen::Ref<const Eigen::Vector2d> &dim, geometry_msgs::Pose pose, const std::string &id)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = dim(0);
  primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = dim(1);

  geometry_msgs::Pose empty_pose;
  empty_pose.orientation.w = 1.0;

  moveit_msgs::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::CollisionObject::ADD;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}
  // ROS_INFO("ADD BOX!!");
  // planning_scene_->getAllowedCollisionMatrixNonConst().setEntry('hand', id);
}

void PlanningSceneCollisionCheck::addSphere(const double &dim, const std::string &id,
            const Eigen::Ref<const Eigen::Vector3d> &pos, const Eigen::Ref<const Eigen::Vector4d> &quat)
{
  geometry_msgs::Pose pose;
  pose = convertEigenToPose(pos, quat);
  addSphere(dim, pose, id);
}

void PlanningSceneCollisionCheck::addSphere(const double &dim, geometry_msgs::Pose pose, const std::string &id)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(1);
  primitive.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = dim;

  geometry_msgs::Pose empty_pose;
  empty_pose.orientation.w = 1.0;

  moveit_msgs::CollisionObject co;
  co.header.frame_id = obs_frame_id_;
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(empty_pose);
  co.pose = pose;
  co.operation = moveit_msgs::CollisionObject::ADD;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}
}

void PlanningSceneCollisionCheck::attachObject(const std::string &object_id, const std::string &link_name, const std::vector<std::string> &touch_links)
{
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.id = object_id;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;
  aco.object.pose.orientation.w = 1.0;
  aco.link_name = link_name;
  aco.touch_links = touch_links;

{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processAttachedCollisionObjectMsg(aco);
}
  // planning_scene_->
}

void PlanningSceneCollisionCheck::detachObject(const std::string &object_id, const std::string &link_name)
{
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.id = object_id;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  aco.object.pose.orientation.w = 1.0;
  aco.link_name = link_name;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processAttachedCollisionObjectMsg(aco);
}
}

void PlanningSceneCollisionCheck::detachAllObjects(const std::string & link_name)
{
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  aco.link_name = link_name;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processAttachedCollisionObjectMsg(aco);
}
}

void PlanningSceneCollisionCheck::removeObject(const std::string & object_id)
{
  moveit_msgs::CollisionObject co;
  co.id = object_id;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}
}

// std::vector<std::string> PlanningSceneCollisionCheck::getAttachedObjects(const std::string & link_name)
// {
  
// }

std::vector<std::string> PlanningSceneCollisionCheck::getAllAttachedObjects()
{
  std::vector<std::string> attached_objects;
  std::vector<moveit_msgs::AttachedCollisionObject> acos;
  planning_scene_->getAttachedCollisionObjectMsgs(acos);
  
  for (auto & aco : acos)
  {
    attached_objects.push_back(aco.object.id);
  }
  return attached_objects;
}

void PlanningSceneCollisionCheck::changeCollision(const std::string &name1, const std::string &name2, bool allowed)
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->getAllowedCollisionMatrixNonConst().setEntry(name1, name2, allowed);
}

void PlanningSceneCollisionCheck::changeCollisions(const std::string &name1, const std::vector< std::string > &other_names, bool allowed)
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->getAllowedCollisionMatrixNonConst().setEntry(name1, other_names, allowed);
}

void PlanningSceneCollisionCheck::changeCollisionsAll(const std::string &name1, bool allowed)
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->getAllowedCollisionMatrixNonConst().setEntry(name1, allowed);
}

Eigen::Isometry3d PlanningSceneCollisionCheck::geometry_pose_to_isometry(geometry_msgs::Pose geometry_pose)
{
  Eigen::Quaterniond quat_;
  Eigen::Isometry3d transform_;
  quat_.x() = geometry_pose.orientation.x;
  quat_.y() = geometry_pose.orientation.y;
  quat_.z() = geometry_pose.orientation.z;
  quat_.w() = geometry_pose.orientation.w;
  transform_.linear() = quat_.toRotationMatrix();
  transform_.translation() << geometry_pose.position.x, geometry_pose.position.y, geometry_pose.position.z;
  return transform_;
}

moveit_msgs::PlanningScene PlanningSceneCollisionCheck::getPlanningSceneMsg()
{
  moveit_msgs::PlanningScene scene_msgs;
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->getPlanningSceneMsg(scene_msgs);
  return scene_msgs;
}

void PlanningSceneCollisionCheck::publishPlanningSceneMsg()
{
  std::scoped_lock _lock(planning_scene_mtx_);
  moveit_msgs::PlanningScene scene_msg;
  // while(ros::ok())
  // {
  //   if (scene_pub_.getNumSubscribers() >= 1) break;
  // }
  planning_scene_->getPlanningSceneMsg(scene_msg);
  scene_pub_.publish(scene_msg);
  // planning_scene_->set
  // planning_scene_->getAllowedCollisionMatrix().print(std::cout);
}

void PlanningSceneCollisionCheck::printCurrentCollisionInfos()
{
  std::cout << streamCurrentCollisionInfos().str();
}
std::stringstream PlanningSceneCollisionCheck::streamCurrentCollisionInfos()
{
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  {
  std::scoped_lock _lock(planning_scene_mtx_);
  robot_state::RobotState current_state = planning_scene_->getCurrentState();
  
  collision_request.contacts = true;
  collision_request.distance = false;
  collision_request.verbose = false;
  planning_scene_->checkCollision(collision_request, collision_result, current_state);
  }
  std::stringstream ss;
  ss << "===================================\n ";
  ss << " > Current collision status: " << collision_result.collision << std::endl;
  for (auto & contact : collision_result.contacts)
  {
    ss << "    > Contact : " << contact.first.first << " - " << contact.first.second << std::endl;
  //             << "      > contact pos :"  << contact.second.
  // Eigen::Vector3d pos;
  }
  ss << "===================================\n ";
  return ss;
}

planning_scene::PlanningScenePtr& PlanningSceneCollisionCheck::getPlanningScene()
{
  return planning_scene_;
}