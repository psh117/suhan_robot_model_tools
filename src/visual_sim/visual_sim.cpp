#include "visual_sim/visual_sim.h"

/**
 * Most of the code in this file is copied from: 
 * Motion Bench Maker (https://github.com/KavrakiLab/motion_bench_maker)
 * Thanks to the authors for making their code available!
*/

VisualSim::VisualSim()
{
   // azure
  cam_props_.width = 512;
  cam_props_.height = 512;
  cam_props_.fx = 550.0;
  cam_props_.fy = 550.0;
  cam_props_.z_near = 0.25;
  cam_props_.z_far = 2.88;
  cam_props_.cx = cam_props_.width / 2.0;
  cam_props_.cy = cam_props_.height / 2.0;
  sim_ = std::make_shared<gds::SimDepthCamera>(cam_props_);

  occupancy_bound_.first << -1.0, -1.0, -1.0;
  occupancy_bound_.second << 1.0, 1.0, 1.0;
}

void VisualSim::setCamPos(const Eigen::Ref<const Eigen::Vector3d> &pos)
{
  cam_pose_.translation() = pos;
}

void VisualSim::setCamPose(const Eigen::Isometry3d &cam_pose)
{
  cam_pose_ = cam_pose;
}

void VisualSim::lookat(const Eigen::Ref<const Eigen::Vector3d> &target)
{
  Eigen::Vector3d z = (target - cam_pose_.translation()).normalized();
  Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitZ()).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  cam_pose_.linear() = rot;
}

void VisualSim::loadScene(const planning_scene::PlanningScenePtr & scene)
{
  const auto &world = scene->getWorld();
  std::vector<std::string> object_ids = world->getObjectIds();
  for (const auto &name : object_ids)
  {
    const auto &obj = world->getObject(name);

    // auto obj = std::make_shared<Geometry>(*obj->shapes_[0]);
    // const auto &obj = scene->getObjectGeometry(name);

    shapes::ShapeConstPtr shape = obj->shapes_[0];
    const auto &mesh = geomToMesh(shape, name);
    sim_->add(name, mesh, obj->pose_);
    // std::cout << "Added object: " << name << std::endl;
    // print obj->shape_poses_[0]
    // std::cout << obj->shape_poses_[0].matrix() << std::endl;
    // std::cout << obj->pose_.matrix() << std::endl;
  }

}

gds::Mesh VisualSim::geomToMesh(const shapes::ShapeConstPtr &shape, 
                                const std::string &name)
{
  gl_depth_sim::EigenAlignedVec<Eigen::Vector3f> vertices;
  std::vector<unsigned int> indices;

  const shapes::Mesh * obj_mesh;

  switch (shape->type)
  {
  case shapes::ShapeType::BOX:
  {
    const auto *box = dynamic_cast<const shapes::Box *>(shape.get());
    obj_mesh = shapes::createMeshFromShape(box);    
    break;
  }
  case shapes::ShapeType::SPHERE:
  {
    const auto *sphere = dynamic_cast<const shapes::Sphere *>(shape.get());
    obj_mesh = shapes::createMeshFromShape(sphere);
    break;
  }
  case shapes::ShapeType::CYLINDER:
  {
    const auto *cylinder = dynamic_cast<const shapes::Cylinder *>(shape.get());
    obj_mesh = shapes::createMeshFromShape(cylinder);
    break;
  }
  case shapes::ShapeType::CONE:
  {
    const auto *cone = dynamic_cast<const shapes::Cone *>(shape.get());
    obj_mesh = shapes::createMeshFromShape(cone);
    break;
  }
  case shapes::ShapeType::MESH:
  {
    obj_mesh = dynamic_cast<const shapes::Mesh *>(shape.get());
    break;
  }
  }

  for (unsigned int i = 0; i < obj_mesh->triangle_count; ++i)
  {
    long unsigned int i3 = i * 3;
    indices.push_back(obj_mesh->triangles[i3]);
    indices.push_back(obj_mesh->triangles[i3 + 1]);
    indices.push_back(obj_mesh->triangles[i3 + 2]);
  }

  for (unsigned int i = 0; i < obj_mesh->vertex_count; ++i)
  {
    long unsigned int i3 = i * 3;
    vertices.push_back({(float)obj_mesh->vertices[i3], (float)obj_mesh->vertices[i3 + 1],
                        (float)obj_mesh->vertices[i3 + 2]});
  }

  auto mesh = gl_depth_sim::Mesh(vertices, indices);
  return mesh;
}

CloudXYZPtr VisualSim::generatePointCloud()
{
  CloudXYZ cloud;
  auto depth_image = sim_->render(cam_pose_);
  gds::toPointCloudXYZ(cam_props_, depth_image, cloud);

  auto tr = cam_pose_.translation();
  // cloud.sensor_origin_ = tr;
  cloud.sensor_origin_.x() = tr.x();
  cloud.sensor_origin_.y() = tr.y();
  cloud.sensor_origin_.z() = tr.z();
  return std::make_shared<CloudXYZ>(cloud);
}

void VisualSim::setGridResolution(const int n_grid)
{
  n_grid_ = n_grid;
}

void VisualSim::setSceneBounds(const Eigen::Ref<const Eigen::Vector3d> &scene_bound_min,
                               const Eigen::Ref<const Eigen::Vector3d> &scene_bound_max)
{
  occupancy_bound_.first = scene_bound_min;
  occupancy_bound_.second = scene_bound_max;
}

Eigen::VectorXi VisualSim::generateVoxelOccupancy()
{
  auto depth_image = sim_->render(cam_pose_);

  const auto &cloud = generatePointCloud();
  
  Eigen::Vector3d occupancy_min, occupancy_max;
  
  for (int i=0; i<3; ++i)
  {
    occupancy_min[i] = occupancy_bound_.first[i];
    occupancy_max[i] = occupancy_bound_.second[i]; 
    length_arr_[i] = occupancy_max[i] - occupancy_min[i];
    n_grids_[i] = n_grid_;
    resolutions_[i] = length_arr_[i] / n_grids_[i];
    
    assert(occupancy_min[i] < occupancy_max[i]);
  }

  voxel_grid.resize(n_grids_[0],n_grids_[1],n_grids_[2]);
  voxel_grid.setZero();

  int cnt_cloud = 0;
  for (const auto &p : *cloud) 
  {
    Eigen::Vector3d global_pos = cam_pose_ * Eigen::Vector3d{p.x, p.y, p.z};
    
    int indices[3];
    bool in_bound = true;
    
    for (int i=0; i<3; ++i)
    {
      if (occupancy_min(i) <= global_pos(i) && 
                              global_pos(i) < occupancy_max(i))
      {
        indices[i] = (global_pos(i) - occupancy_min[i]) / resolutions_[i];

        assert(indices[i] < n_grids_[i]);
        assert(indices[i] >= 0);
      }
      else
      {
        in_bound = false;
        break;
      }
    }

    if(in_bound)
    { 
      voxel_grid(indices[0],indices[1],indices[2]) = 1;              
    }
  }

  Eigen::VectorXi voxel_vector;
  voxel_vector.resize(n_grid_ * n_grid_ * n_grid_);

  for(int i=0; i<n_grid_; i++)
  {
    for(int j=0; j<n_grid_; j++)
    {
      for(int k=0; k<n_grid_; k++)
      {
        voxel_vector[i*n_grid_*n_grid_ + j*n_grid_ + k] = voxel_grid(i, j, k);
      } 
    } 
  }
  
  return voxel_vector;
}


Eigen::MatrixXf VisualSim::generateDepthImage()
{
  auto depth_image = sim_->render(cam_pose_);
  
  Eigen::MatrixXf depth_image_eigen(cam_props_.height, cam_props_.width);
  for (int i = 0; i < cam_props_.height; ++i)
  {
    for (int j = 0; j < cam_props_.width; ++j)
    {
      depth_image_eigen(i, j) = depth_image.distance(i, j);
    }
  }
  return depth_image_eigen;
}

