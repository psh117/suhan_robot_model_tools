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

Eigen::MatrixXd VisualSim::generatePointCloudMatrix()
{
  const auto &cloud = generatePointCloud();
  Eigen::MatrixXd cloud_matrix(cloud->size(), 3);
  for (int i = 0; i < cloud->size(); ++i)
  {
    cloud_matrix(i, 0) = (*cloud)[i].x;
    cloud_matrix(i, 1) = (*cloud)[i].y;
    cloud_matrix(i, 2) = (*cloud)[i].z;
  }
  return cloud_matrix;
}

void VisualSim::setGridResolution(const int n_grid)
{
  n_grid_ = n_grid;
  for (int i=0; i<3; ++i)
  {
    n_grids_[i] = n_grid;
  }
}

void VisualSim::setGridResolutions(const int n_grid_x, const int n_grid_y, const int n_grid_z)
{
  n_grids_[0] = n_grid_x;
  n_grids_[1] = n_grid_y;
  n_grids_[2] = n_grid_z;
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
    resolutions_[i] = length_arr_[i] / n_grids_[i];
    
    assert(occupancy_min[i] < occupancy_max[i]);
  }

  voxel_grid.resize(n_grids_[0],n_grids_[1],n_grids_[2]);
  voxel_grid.setZero();

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
  voxel_vector.resize(n_grids_[0] * n_grids_[1] * n_grids_[2]);

  for(int i=0; i<n_grids_[0]; i++)
  {
    for(int j=0; j<n_grids_[1]; j++)
    {
      for(int k=0; k<n_grids_[2]; k++)
      {
        voxel_vector[i*n_grids_[0]*n_grids_[1] + j*n_grids_[1] + k] = voxel_grid(i, j, k);
      } 
    } 
  }
  
  return voxel_vector;
}


Eigen::VectorXi VisualSim::generateLocalVoxelOccupancy(const Eigen::MatrixXd &point_cloud_matrix, 
                                                       const Eigen::Isometry3d& obj_pose,
                                                       const Eigen::Ref<const Eigen::Vector3d> &obj_bound_min_vec,
                                                       const Eigen::Ref<const Eigen::Vector3d> &obj_bound_max_vec,
                                                       const Eigen::Ref<const Eigen::Vector3d> &n_grids_vec, 
                                                       double near_distance,
                                                       bool fill_occluded_voxels)
{
  auto &obj_bound_min = obj_bound_min_vec.array();
  auto &obj_bound_max = obj_bound_max_vec.array();
  auto &n_grids = n_grids_vec.array().cast<int>();

  Eigen::Array3i obj_bound_idx_min, obj_bound_idx_max;
  Eigen::Array3d lengths, resolutions, resolutions_inv;
  Eigen::Array3d local_occu_bounding_coord_min, local_occu_bounding_coord_max;

  local_occu_bounding_coord_min = obj_bound_min - near_distance;
  local_occu_bounding_coord_max = obj_bound_max + near_distance;

  lengths = local_occu_bounding_coord_max - local_occu_bounding_coord_min;
  resolutions = lengths / n_grids.cast<double>();
  resolutions_inv = 1. / resolutions;

  obj_bound_idx_min = ((obj_bound_min - local_occu_bounding_coord_min) * resolutions_inv).cast<int>();
  obj_bound_idx_max = ((obj_bound_max - local_occu_bounding_coord_min) * resolutions_inv).cast<int>();

  assert((local_occu_bounding_coord_min < local_occu_bounding_coord_max).all());

  voxel_grid.resize(n_grids[0],n_grids[1],n_grids[2]);
  voxel_grid.setZero();

  Eigen::Array3d obj_camera_point = obj_pose.inverse() * cam_pose_.translation();
  for (int i=0; i<point_cloud_matrix.rows(); ++i) 
  {
    Eigen::Vector3d point = point_cloud_matrix.row(i).transpose();
    Eigen::Vector3d global_point = cam_pose_ * point;
    Eigen::Array3d obj_point = obj_pose.inverse() * global_point;
    
    bool out_of_bound = true;

    if (obj_point.hasNaN()) continue;

    if ((obj_point < local_occu_bounding_coord_max).all() &&
        (obj_point > local_occu_bounding_coord_min).all())
    {
      out_of_bound = false;
    }

    if (out_of_bound) continue;
    
    Eigen::Array3i indices;

    indices = ((obj_point - local_occu_bounding_coord_min) * resolutions_inv).cast<int>();

    assert((indices < n_grids).all());
    assert((indices >= 0).all());

    voxel_grid(indices[0],indices[1],indices[2]) = 1;              

    if (fill_occluded_voxels)
    {
        Eigen::Vector3d direction = obj_point - obj_camera_point;
        auto origin = obj_point;
        
        direction = direction.normalized(); // normalize vector
        int    current_key[3];
        int    step[3];
        double t_max[3];
        double t_delta[3];

        for(unsigned int i=0; i < 3; ++i) 
        {
          current_key[i] = indices[i];

          // compute step direction
          if (direction(i) > 0.0)        step[i] =  1;
          else if (direction(i) < 0.0)   step[i] = -1;
          else                           step[i] = 0;

          // compute t_max, t_delta
          if (step[i] != 0) 
          {
              double voxel_border = (indices[i] * resolutions[i]) + local_occu_bounding_coord_min[i];
              voxel_border += (float) (step[i] * resolutions[i] * 0.5);

              t_max[i] = ( voxel_border - origin(i) ) / direction(i);
              t_delta[i] = resolutions[i] / fabs( direction(i) );
          }
          else 
          {
              t_max[i] =  std::numeric_limits<double>::max( );
              t_delta[i] = std::numeric_limits<double>::max( );
          }
      }

      bool done = false;
      while (!done) 
      {
        unsigned int dim;

        // find minimum t_max:
        if (t_max[0] < t_max[1])
        {
            if (t_max[0] < t_max[2]) dim = 0;
            else                     dim = 2;
        }
        else 
        {
            if (t_max[1] < t_max[2]) dim = 1;
            else                     dim = 2;
        }

        current_key[dim] += step[dim];
        t_max[dim] += t_delta[dim];

        if (current_key[dim] >= n_grids[dim] || current_key[dim] < 0) 
        {
          done = true;
          break;
        }

        double dist_from_origin = std::min(std::min(t_max[0], t_max[1]), t_max[2]);
        
        assert(current_key[0] >= 0);
        assert(current_key[1] >= 0);
        assert(current_key[2] >= 0);
        assert(current_key[0] < n_grids[0]);
        assert(current_key[1] < n_grids[1]);
        assert(current_key[2] < n_grids[2]);
        
        voxel_grid(current_key[0],current_key[1],current_key[2]) = 2;                
      } 
    } // end if of fill_occluded_voxels
  } // end for (int i=0; i<point_cloud_matrix.rows(); ++i)

  // mark object pose in voxel grid
  for (int i=obj_bound_idx_min[0]; i<=obj_bound_idx_max[0]; ++i) 
  {
    for (int j=obj_bound_idx_min[1]; j<=obj_bound_idx_max[1]; ++j) 
    {
      for (int k=obj_bound_idx_min[2]; k<=obj_bound_idx_max[2]; ++k) 
      {
        if (voxel_grid(i,j,k) == 0) continue;
        voxel_grid(i,j,k) = - voxel_grid(i,j,k);
      }
    }
  }
  // write voxel in vector
  Eigen::VectorXi voxel_vector;
  voxel_vector.resize(n_grids[0] * n_grids[1] * n_grids[2]);

  for(int i=0; i<n_grids[0]; i++)
  {
    for(int j=0; j<n_grids[1]; j++)
    {
      for(int k=0; k<n_grids[2]; k++)
      {
        voxel_vector[i*n_grids[0]*n_grids[1] + j*n_grids[1] + k] = voxel_grid(i, j, k);
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

