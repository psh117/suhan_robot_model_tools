#include "robot_model/planning_scene_collision_check.h"

PlanningSceneCollisionCheck::PlanningSceneCollisionCheck()
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();
  planning_scene_ = std::make_shared<planning_scene::PlanningScene> (robot_model_);
  scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scenes_suhan", 1);
  // acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(planning_scene_->getAllowedCollisionMatrix());
};

void PlanningSceneCollisionCheck::setArmNames(const std::vector<std::string> &arm_name)
{
  arm_name_ = arm_name;  
}

bool PlanningSceneCollisionCheck::isValid(const Eigen::Ref<const Eigen::VectorXd> &q) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  robot_state::RobotState current_state = planning_scene_->getCurrentState();
  
  for (int i=0; i<arm_name_.size(); i++)
  {
    const auto & q_seg = q.segment<7>(i*7);
    const std::vector<double> joint_values(q_seg.data(), q_seg.data() + 7);
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(arm_name_[i]);
    current_state.setJointGroupPositions(joint_model_group, joint_values);
  }
  Eigen::Vector2d gpos;
  gpos << 0.04, 0.04;
  current_state.setJointGroupPositions("hand_left", gpos);
  current_state.setJointGroupPositions("hand_right", gpos);
  current_state.setJointGroupPositions("hand_top", gpos);
  planning_scene_->checkCollision(collision_request, collision_result, current_state);

  last_collision_result_ = collision_result; 
  // DEBUG_FILE("q: " << q.transpose());
  // DEBUG_FILE("validity: " << !collision_result.collision);
  // collision_result->
  // planning_scene_->checkCollision()

  return !collision_result.collision;
}

double PlanningSceneCollisionCheck::clearance(const Eigen::Ref<const Eigen::VectorXd> &q) const
{
  std::scoped_lock _lock(planning_scene_mtx_);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.distance = true;
  robot_state::RobotState current_state = planning_scene_->getCurrentState();
  
  for (int i=0; i<arm_name_.size(); i++)
  {
    const auto & q_seg = q.segment<7>(i*7);
    const std::vector<double> joint_values(q_seg.data(), q_seg.data() + 7);
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(arm_name_[i]);
    current_state.setJointGroupPositions(joint_model_group, joint_values);
  }

  Eigen::Vector2d gpos;
  gpos << 0.04, 0.04;
  current_state.setJointGroupPositions("hand_left", gpos);
  current_state.setJointGroupPositions("hand_right", gpos);
  current_state.setJointGroupPositions("hand_top", gpos);
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
  for (int i=0; i<arm_name_.size(); i++)
  {
    Eigen::Matrix<double, 7, 1> q_seg = q.segment<7>(i*7);
    // std::vector<double> joint_values(q_seg.data(), q_seg.data() + 7);
    // const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup(arm_name_[i]);
    // current_state.setJointGroupPositions(joint_model_group, joint_values);
    // std::cout << arm_name_[i] << ": " << q_seg.transpose() << std::endl;
    current_state.setJointGroupPositions(arm_name_[i], q_seg);
  };
  Eigen::Vector2d gpos;
  gpos << 0.04, 0.04;
  current_state.setJointGroupPositions("hand_left", gpos);
  current_state.setJointGroupPositions("hand_right", gpos);
  current_state.setJointGroupPositions("hand_top", gpos);
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
  co.header.frame_id = "/base";
  co.id = id;
  co.meshes.push_back(shape_msgs_mesh);
  co.mesh_poses.push_back(pose);
  co.operation = moveit_msgs::CollisionObject::ADD;

{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}

}

void PlanningSceneCollisionCheck::updateObjectPose(geometry_msgs::Pose pose, const std::string &id)
{
  moveit_msgs::CollisionObject co;
  co.header.frame_id = "/base";
  co.id = id;
  co.mesh_poses.push_back(pose);
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

  moveit_msgs::CollisionObject co;
  co.header.frame_id = "/base";
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
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

  moveit_msgs::CollisionObject co;
  co.header.frame_id = "/base";
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
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

  moveit_msgs::CollisionObject co;
  co.header.frame_id = "/base";
  co.id = id;
  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
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
  aco.link_name = link_name;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processAttachedCollisionObjectMsg(aco);
}
}

void PlanningSceneCollisionCheck::detachAllObject(const std::string & link_name)
{
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;
  aco.link_name = link_name;
  
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processAttachedCollisionObjectMsg(aco);
}
}

void PlanningSceneCollisionCheck::deleteObject(const std::string & object_id)
{
  moveit_msgs::CollisionObject co;
  co.id = object_id;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
{
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->processCollisionObjectMsg(co);
}
}

std::vector<std::string> PlanningSceneCollisionCheck::getAttachedObjects(const std::string & link_name)
{
  
}

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

void PlanningSceneCollisionCheck::cellChecker(const std::string object_id, const bool visual, Eigen::Ref<Eigen::VectorXd> occupancy_map)
{
  Eigen::Vector3d mat_size, pos, new_size;
  Eigen::Vector4d quat;
  std::cout << "occupancy map - base object id: " << object_id << std::endl;
  if (object_id == "world")
  {
    std::cout << "occupancy check in world frame" << std::endl;
    mat_size << 60, 60, 40;
    new_size << 3.0, 3.0, 2.0;
    pos << -1.0, -1.5, 0.9;
    quat << 0, 0, 0, 1;
    cellChecker(pos, quat, mat_size, new_size, visual,object_id, occupancy_map);
  }
  else
  {
    mat_size << 16, 16, 16;
    moveit_msgs::PlanningScene scene_msgs;
    {
    std::scoped_lock _lock(planning_scene_mtx_);
    planning_scene_ -> getPlanningSceneMsg(scene_msgs);
    }
    Eigen::Isometry3d T_base_object;
    double additional_length = 0.3;
    double max_x = 0.; double max_y = 0.; double max_z = 0.;
    double min_x = 999.; double min_y = 999.; double min_z = 999.;
    for (moveit_msgs::CollisionObject & collision_object_ : scene_msgs.world.collision_objects)
    {
      if (collision_object_.id == object_id)
      {
        if (collision_object_.meshes.size() > 0)
        {
          T_base_object = geometry_pose_to_isometry(collision_object_.mesh_poses[0]);
          for (geometry_msgs::Point & point_ : collision_object_.meshes[0].vertices)
          { 
            if (point_.x > max_x) {max_x = point_.x;}
            if (point_.x < min_x) {min_x = point_.x;}
            if (point_.y > max_y) {max_y = point_.y;}
            if (point_.y < min_y) {min_y = point_.y;}
            if (point_.z > max_z) {max_z = point_.z;}
            if (point_.z < min_z) {min_z = point_.z;}
          }
        }
        if (collision_object_.primitives.size() > 0)
        {
          T_base_object = geometry_pose_to_isometry(collision_object_.primitive_poses[0]);
          shape_msgs::SolidPrimitive & primitive_ = collision_object_.primitives[0];
          if(primitive_.type == primitive_.BOX)
          {
            max_x = primitive_.dimensions[primitive_.BOX_X]/2;
            max_y = primitive_.dimensions[primitive_.BOX_Y]/2;
            max_z = primitive_.dimensions[primitive_.BOX_Z]/2;
            min_x = -primitive_.dimensions[primitive_.BOX_X]/2;
            min_y = -primitive_.dimensions[primitive_.BOX_Y]/2;
            min_z = -primitive_.dimensions[primitive_.BOX_Z]/2;
          }

          else if (primitive_.type == primitive_.SPHERE)
          {
            max_x = primitive_.dimensions[primitive_.SPHERE_RADIUS]/2;
            max_y = primitive_.dimensions[primitive_.SPHERE_RADIUS]/2;
            max_z = primitive_.dimensions[primitive_.SPHERE_RADIUS]/2;
            min_x = -primitive_.dimensions[primitive_.SPHERE_RADIUS]/2;
            min_y = -primitive_.dimensions[primitive_.SPHERE_RADIUS]/2;
            min_z = -primitive_.dimensions[primitive_.SPHERE_RADIUS]/2;
          }

          else if (primitive_.type == primitive_.CYLINDER)
          {
            max_x = primitive_.dimensions[primitive_.CYLINDER_RADIUS]/2;
            max_y = primitive_.dimensions[primitive_.CYLINDER_RADIUS]/2;
            max_z = primitive_.dimensions[primitive_.CYLINDER_HEIGHT]/2;
            min_x = -primitive_.dimensions[primitive_.CYLINDER_RADIUS]/2;
            min_y = -primitive_.dimensions[primitive_.CYLINDER_RADIUS]/2;
            min_z = -primitive_.dimensions[primitive_.CYLINDER_HEIGHT]/2;
          }

          else if (primitive_.type == primitive_.CONE)
          {
            max_x = primitive_.dimensions[primitive_.CONE_RADIUS]/2;
            max_y = primitive_.dimensions[primitive_.CONE_RADIUS]/2;
            max_z = primitive_.dimensions[primitive_.CONE_HEIGHT]/2;
            min_x = -primitive_.dimensions[primitive_.CONE_RADIUS]/2;
            min_y = -primitive_.dimensions[primitive_.CONE_RADIUS]/2;
            min_z = -primitive_.dimensions[primitive_.CONE_HEIGHT]/2;
          }
        }
        pos << min_x - additional_length/2, min_y - additional_length/2, min_z - additional_length/2;
        pos << T_base_object * pos;
        Eigen::Quaterniond q_(T_base_object.linear());
        quat << q_.x(), q_.y(), q_.z(), q_.w();
        new_size << additional_length + max_x - min_x, additional_length + max_y - min_y, additional_length + max_z - min_z;
        cellChecker(pos, quat, mat_size, new_size, visual, object_id, occupancy_map);
        break;
      }
    }
  }
}

void PlanningSceneCollisionCheck::cellChecker(const Eigen::Ref<const Eigen::Vector3d> &pos,
                                              const Eigen::Ref<const Eigen::Vector4d> &quat,
                                              const Eigen::Ref<const Eigen::Vector3d> &mat_size,
                                              const Eigen::Ref<const Eigen::Vector3d> &world_size,
                                              const bool visual, const std::string object_id, Eigen::Ref<Eigen::VectorXd> occupancy_map)
{
  bool debug = true;
  moveit_msgs::PlanningScene scene_msgs;
  Eigen::Isometry3d T_WM_, T_MV_, T_WV_, T_world;
  Eigen::Vector3d point_;
  T_MV_.setIdentity();
  T_world.translation() = pos;
  T_world.linear() = Eigen::Quaterniond(quat).toRotationMatrix();
  T_world = T_world.inverse();

  {
  std::scoped_lock _lock(planning_scene_mtx_);
  planning_scene_->getPlanningSceneMsg(scene_msgs);
  }

  const int OBJECT = 1;
  const int OBSTACLE = 2;
  const int ROBOT = 3;
  int mat_x = mat_size(0);
  int mat_y = mat_size(1);
  int mat_z = mat_size(2);
  // occupancy_map.resize(mat_x*mat_y*mat_z);
  // occupancy_map.setZero();
  double cube_x = world_size(0)/mat_x;
  double cube_y = world_size(1)/mat_y;
  double cube_z = world_size(2)/mat_z;
  double step_size_x = 0.4 * world_size(0) / mat_x;
  double step_size_y = 0.4 * world_size(1) / mat_y;
  double step_size_z = 0.4 * world_size(2) / mat_z;
  double search_length = 0.01;
  Eigen::Tensor<int, 3> return_matrix(mat_x, mat_y, mat_z);
  return_matrix.setZero();

  if (visual)
  {
    std::cout << "\nmat_size x: " << mat_x << std::endl;
    std::cout << "mat_size y: " << mat_y << std::endl;
    std::cout << "mat_size z: " << mat_z << std::endl;
    std::cout << "grid length x: " << cube_x << std::endl;
    std::cout << "grid length y: " << cube_y << std::endl;
    std::cout << "grid length z: " << cube_z << std::endl;
  }

  // if (debug) std::cout << "DEBUG -- reading mesh" << std::endl;
  for (moveit_msgs::CollisionObject & collision_object_ : scene_msgs.world.collision_objects)
  {
    if(collision_object_.mesh_poses.size()>0) // mesh
    {
      for (int i=0;i<collision_object_.mesh_poses.size();i++)
      {
        std::vector<Eigen::Vector3d> vertices;
        T_WM_ = T_world * geometry_pose_to_isometry(collision_object_.mesh_poses[i]); // calculate mesh transform
        for (geometry_msgs::Point & point_ : collision_object_.meshes[i].vertices)
        {
          T_MV_.translation() << point_.x, point_.y, point_.z;
          vertices.push_back((T_WM_ * T_MV_).translation()); // vertex point in world frame
        }
        for (int idx_=0; idx_<collision_object_.meshes[i].triangles.size(); idx_++)
        {
          Eigen::Vector3d vertex0 = vertices[collision_object_.meshes[i].triangles[idx_].vertex_indices[0]];
          Eigen::Vector3d vector1 = vertices[collision_object_.meshes[i].triangles[idx_].vertex_indices[1]] - vertex0;
          Eigen::Vector3d vector2 = vertices[collision_object_.meshes[i].triangles[idx_].vertex_indices[2]] - vertex0;
          double rate_step1 = search_length / vector1.norm();
          double rate_step2 = search_length / vector2.norm();
          if (vector1.norm() < search_length * 0.5) {rate_step1 = 0.45;}
          if (vector2.norm() < search_length * 0.5) {rate_step2 = 0.45;}

          double rate1 = 0.0;
          while (rate1 < 1.0 + rate_step1)
          {
            double rate2 = 0.0;
            while (rate2 < 1.0 + rate_step2 - rate1)
            {
              point_ = vertex0 + rate1 * vector1 + rate2 * vector2;
              int ii = (point_(0)) / cube_x;
              int jj = (point_(1)) / cube_y;
              int kk = (point_(2)) / cube_z;
              if(ii>-1 && jj>-1 && kk>-1 && ii<mat_x && jj<mat_y && kk<mat_z)
              {
                if (collision_object_.id == object_id) {return_matrix(ii,jj,kk) = OBJECT;}
                if (return_matrix(ii,jj,kk) != OBJECT) {return_matrix(ii,jj,kk) = OBSTACLE;}
              }
              rate2 += rate_step2;
            }
            rate1 += rate_step1;
          }
        }
      }
    }

    // if (debug) std::cout << "DEBUG -- reading primitive" << std::endl;
    if(collision_object_.primitive_poses.size()>0)
    {
      double x, y, z;
      for (int i=0; i<collision_object_.primitive_poses.size(); i++)
      {
        T_WM_ = T_world * geometry_pose_to_isometry(collision_object_.primitive_poses[i]);
        shape_msgs::SolidPrimitive & primitive_ = collision_object_.primitives[i];
        if(primitive_.type == primitive_.BOX) // box
        {
          x = -primitive_.dimensions[primitive_.BOX_X]/2;
          while (x <= primitive_.dimensions[primitive_.BOX_X]/2)
          {
            y = -primitive_.dimensions[primitive_.BOX_Y]/2;
            while (y <= primitive_.dimensions[primitive_.BOX_Y]/2)
            {
              z = -primitive_.dimensions[primitive_.BOX_Z]/2;
              while (z <= primitive_.dimensions[primitive_.BOX_Z]/2)
              {
                T_MV_.translation() << x, y, z;
                point_ = (T_WM_ * T_MV_).translation();
                int i = (point_(0)) / cube_x;
                int j = (point_(1)) / cube_y;
                int k = (point_(2)) / cube_z;
                if(i>-1 && j>-1 && k>-1 && i<mat_x && j<mat_y && k<mat_z)
                {
                  if (collision_object_.id == object_id) {return_matrix(i,j,k) = OBJECT;}
                  if (return_matrix(i,j,k) != OBJECT) {return_matrix(i,j,k) = OBSTACLE;}
                }
                z += step_size_z;
              }
              y += step_size_y;
            }
            x += step_size_x;
          }
        }

        else if (primitive_.type == primitive_.SPHERE)
        {
          double radius_step = sqrt(pow(step_size_y,2) + pow(step_size_z,2));
          z = -primitive_.dimensions[primitive_.SPHERE_RADIUS]/2;
          double max_radius = primitive_.dimensions[primitive_.SPHERE_RADIUS];
          while (z <= primitive_.dimensions[primitive_.SPHERE_RADIUS]/2)
          {
            double radius = 0.0;
            double radius_limit = sqrt(pow(max_radius,2) - pow(abs(z),2));
            while (radius <= radius_limit)
            {
              double angle = 0.0;
              while (angle <= 2*M_PI)
              {
                T_MV_.translation() << radius * cos(angle), radius * sin(angle), z;
                point_ = (T_WM_ * T_MV_).translation();
                int i = (point_(0)) / cube_x;
                int j = (point_(1)) / cube_y;
                int k = (point_(2)) / cube_z;
                if(i>-1 && j>-1 && k>-1 && i<mat_x && j<mat_y && k<mat_z)
                {
                  if (collision_object_.id == object_id) {return_matrix(i,j,k) = OBJECT;}
                  if (return_matrix(i,j,k) != OBJECT) {return_matrix(i,j,k) = OBSTACLE;}
                }
                angle += M_PI * 0.05;
              }
              radius += radius_step;
            }
            z += step_size_z;
          }
        }

        else if (primitive_.type == primitive_.CYLINDER)
        {
          double radius_step = sqrt(pow(step_size_y,2) + pow(step_size_z,2));
          z = -primitive_.dimensions[primitive_.CYLINDER_HEIGHT]/2;
          while (z <= primitive_.dimensions[primitive_.CYLINDER_HEIGHT]/2)
          {
            double radius = 0.0;
            while (radius <= primitive_.dimensions[primitive_.CYLINDER_RADIUS])
            {
              double angle = 0.0;
              while (angle <= 2*M_PI)
              {
                T_MV_.translation() << radius * cos(angle), radius * sin(angle), z;
                point_ = (T_WM_ * T_MV_).translation();
                int i = (point_(0)) / cube_x;
                int j = (point_(1)) / cube_y;
                int k = (point_(2)) / cube_z;
                if(i>-1 && j>-1 && k>-1 && i<mat_x && j<mat_y && k<mat_z)
                {
                  if (collision_object_.id == object_id) {return_matrix(i,j,k) = OBJECT;}
                  if (return_matrix(i,j,k) != OBJECT) {return_matrix(i,j,k) = OBSTACLE;}
                }
                angle += M_PI * 0.05;
              }
              radius += radius_step;
            }
            z += step_size_z;
          }
        }

        else if (primitive_.type == primitive_.CONE)
        {
          double radius_step = sqrt(pow(step_size_y,2) + pow(step_size_z,2));
          z = -primitive_.dimensions[primitive_.CONE_HEIGHT]/2;
          double max_radius = primitive_.dimensions[primitive_.CONE_RADIUS];
          while (z <= primitive_.dimensions[primitive_.CONE_HEIGHT]/2)
          {
            double radius = 0.0;
            double radius_limit = max_radius * (primitive_.dimensions[primitive_.CONE_HEIGHT]/2 - z) / primitive_.dimensions[primitive_.CONE_HEIGHT];
            while (radius <= radius_limit)
            {
              double angle = 0.0;
              while (angle <= 2*M_PI)
              {
                T_MV_.translation() << radius * cos(angle), radius * sin(angle), z;
                point_ = (T_WM_ * T_MV_).translation();
                int i = (point_(0)) / cube_x;
                int j = (point_(1)) / cube_y;
                int k = (point_(2)) / cube_z;
                if(i>-1 && j>-1 && k>-1 && i<mat_x && j<mat_y && k<mat_z)
                {
                  if (collision_object_.id == object_id) {return_matrix(i,j,k) = OBJECT;}
                  if (return_matrix(i,j,k) != OBJECT) {return_matrix(i,j,k) = OBSTACLE;}
                }
                angle += M_PI * 0.05;
              }
              radius += radius_step;
            }
            z += step_size_z;
          }
        }
      }
    }
  }

  if (debug) std::cout << "DEBUG -- checking robot - making box" << std::endl;
  //////////////// ROBOT CHECK ////////////////
  Eigen::Vector4d qu_(0.0, 0.0, 0.0, 1.0);
  double shrink = 0.001;
  for(int a=0; a<mat_x; a++)
  {
    for(int b=0; b<mat_y; b++)
    {
      for (int c=0; c<mat_z; c++)
      {
        T_WM_.setIdentity();
        T_WM_.translation() << (a+0.5)*cube_x, (b+0.5)*cube_y, (c+0.5)*cube_z;
        T_WM_ = T_world.inverse() * T_WM_;
        Eigen::Quaterniond q_(T_WM_.linear());
        qu_ << q_.x(), q_.y(), q_.z(), q_.w();
        addBox(Eigen::Vector3d(cube_x-shrink, cube_y-shrink, cube_z-shrink), ("box_matrix_" + std::to_string(a) + "_" + std::to_string(b) + "_" + std::to_string(c)), T_WM_.translation(), qu_);
        ros::Duration(0.0005).sleep();
      }
    }
  }
  // publishPlanningSceneMsg();
  ros::Duration(0.001).sleep();
  if (debug) std::cout << "DEBUG -- checking robot - collision checking" << std::endl;
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.max_contacts = mat_x * mat_y * mat_z;
  {
  std::scoped_lock _lock(planning_scene_mtx_);
  robot_state::RobotState current_state = planning_scene_->getCurrentState();
  planning_scene_->checkCollision(collision_request, collision_result, current_state);
  }
  if (collision_result.collision)
  {
    for (auto & contact : collision_result.contacts)
    {
      for(int a=0; a<mat_x; a++)
      {
        for(int b=0; b<mat_y; b++)
        {
          for (int c=0; c<mat_z; c++)
          {
            if (contact.first.first == ("box_matrix_" + std::to_string(a) + "_" + std::to_string(b) + "_" + std::to_string(c)) || contact.first.second == ("box_matrix_" + std::to_string(a) + "_" + std::to_string(b) + "_" + std::to_string(c)))
            {
              return_matrix(a,b,c) = ROBOT;
              break;
            }
          }
        }
      }
    }
  }

  ros::Duration(0.001).sleep();
  if (debug) std::cout << "DEBUG -- checking robot - removing box" << std::endl;
  for(int a=0; a<mat_x; a++)
  {
    for(int b=0; b<mat_y; b++)
    {
      for (int c=0; c<mat_z; c++)
      {
        deleteObject(("box_matrix_" + std::to_string(a) + "_" + std::to_string(b) + "_" + std::to_string(c)));
        ros::Duration(0.0005).sleep();
      }
    }
  }

  if (debug) std::cout << "DEBUG -- checking robot - matrix to vector" << std::endl;
  int name_maker_ = 0;
  double sphere_radius = sqrt(pow(cube_x,2) + pow(cube_y,2) + pow(cube_z,2)) / 2;
  double cylinder_radius = sqrt(pow(cube_x,2) + pow(cube_y,2)) / 2;
  for(int a=0; a<mat_x; a++){
    for(int b=0; b<mat_y; b++){
      for(int c=0; c<mat_z; c++){
        if (return_matrix(a,b,c) > 0)
        {
          if (visual)
          {
            T_WM_.setIdentity();
            T_WM_.translation() << (a+0.5)*cube_x, (b+0.5)*cube_y, (c+0.5)*cube_z;
            T_WM_ = T_world.inverse() * T_WM_;
            Eigen::Quaterniond q_(T_WM_.linear());
            qu_ << q_.x(), q_.y(), q_.z(), q_.w();
            if (return_matrix(a,b,c) == OBSTACLE) addBox(Eigen::Vector3d(cube_x,cube_y,cube_z), std::to_string(name_maker_), T_WM_.translation(), qu_);
            else if (return_matrix(a,b,c) == OBJECT) addCylinder(Eigen::Vector2d(cube_z, cylinder_radius), std::to_string(name_maker_), T_WM_.translation(), qu_);
            else if (return_matrix(a,b,c) == ROBOT) addSphere(sphere_radius, std::to_string(name_maker_), T_WM_.translation(), qu_);
            name_maker_++;
          }
          int idx = c + b*mat_z + a*mat_y*mat_z;
          occupancy_map(idx) = return_matrix(a,b,c);
        }
      }
    }
  }
  if (visual)
  {
    publishPlanningSceneMsg();
    ros::Duration(1.0).sleep();
    for (int i=0; i<name_maker_; i++)
      deleteObject(std::to_string(i));
    publishPlanningSceneMsg();
  }
  if (debug) std::cout << "DEBUG -- END OF LOOP" << std::endl;
}

bool PlanningSceneCollisionCheck::getArmInCollision(const std::string & arm_name, std::string & collision_arm)
{
  collision_detection::CollisionResult collision_result = last_collision_result_;
  if (collision_result.collision)
  {
    for (auto & contact : collision_result.contacts)
    {
      std::string collision_link;
      if (contact.first.first.find(arm_name) != std::string::npos)
      {
        collision_link = contact.first.second;
      }
      else if (contact.first.second.find(arm_name) != std::string::npos)
      {
        collision_link = contact.first.first;
      }

      DEBUG_FILE("contact.first.first: " << contact.first.first);
      DEBUG_FILE("contact.first.second: " << contact.first.second);
      DEBUG_FILE("collision_link: " << collision_link);

      int pos;
      pos = collision_link.find("_hand");
      if (pos != std::string::npos)
      {
        collision_arm = collision_link.substr(0,pos);
        return true;
      }
      pos = collision_link.find("_finger");
      if (pos != std::string::npos)
      {
        collision_arm = collision_link.substr(0,pos);
        return true;
      }
      pos = collision_link.find("_leftfinger");
      if (pos != std::string::npos)
      {
        collision_arm = collision_link.substr(0,pos);
        return true;
      }
      pos = collision_link.find("_rightfinger");
      if (pos != std::string::npos)
      {
        collision_arm = collision_link.substr(0,pos);
        return true;
      }
      pos = collision_link.find("_link");
      if (pos != std::string::npos)
      {
        collision_arm = collision_link.substr(0,pos);
        return true;
      }
    }
  }
  return false;
}

std::string PlanningSceneCollisionCheck::getCollidingArm(const std::vector<std::string> & arm_names)
{
  moveit_msgs::AttachedCollisionObject attached_collision_obj;
  
  collision_detection::CollisionResult collision_result = last_collision_result_;
  if (collision_result.collision)
  {
    for (auto & contact : collision_result.contacts)
    {
      std::string collision_link;
      if (planning_scene_->getAttachedCollisionObjectMsg(attached_collision_obj, contact.first.first))
      {
        collision_link = attached_collision_obj.link_name;
        for (auto & arm_name : arm_names)
        {
          if (collision_link.find(arm_name) != std::string::npos)
          {
            return arm_name;
          }
          else if (collision_link.find(arm_name) != std::string::npos)
          {
            return arm_name;
          }
        }
      }
      if (planning_scene_->getAttachedCollisionObjectMsg(attached_collision_obj, contact.first.second))
      {
        collision_link = attached_collision_obj.link_name;
        for (auto & arm_name : arm_names)
        {
          if (collision_link.find(arm_name) != std::string::npos)
          {
            return arm_name;
          }
          else if (collision_link.find(arm_name) != std::string::npos)
          {
            return arm_name;
          }
        }
      }

      for (auto & arm_name : arm_names)
      {
        if (contact.first.first.find(arm_name) != std::string::npos)
        {
          return arm_name;
        }
        else if (contact.first.second.find(arm_name) != std::string::npos)
        {
          return arm_name;
        }
      }
    }
  }
  throw std::out_of_range("No arm in collision!");
  return "";
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
  Eigen::Vector2d gpos;
  gpos << 0.04, 0.04;
  current_state.setJointGroupPositions("hand_left", gpos);
  current_state.setJointGroupPositions("hand_right", gpos);
  current_state.setJointGroupPositions("hand_top", gpos);
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