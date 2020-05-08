/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/robot_model/link_model.h>
#include <moveit/robot_model/joint_model.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_model/aabb.h>
#include <moveit/exceptions/exceptions.h>

namespace moveit
{
namespace core
{
LinkModel::LinkModel(const std::string& name)
  : name_(name)
  , parent_joint_model_(nullptr)
  , parent_link_model_(nullptr)
  , is_parent_joint_fixed_(false)
  , joint_origin_transform_is_identity_(true)
  , first_collision_body_transform_index_(-1)
  , link_index_(-1)
{
  joint_origin_transform_.setIdentity();
}

LinkModel::~LinkModel() = default;

void LinkModel::setJointOriginTransform(const Eigen::Isometry3d& transform)
{
  joint_origin_transform_ = transform;
  joint_origin_transform_is_identity_ =
      joint_origin_transform_.rotation().isIdentity() &&
      joint_origin_transform_.translation().norm() < std::numeric_limits<double>::epsilon();
}

void LinkModel::setParentJointModel(const JointModel* joint)
{
  parent_joint_model_ = joint;
  is_parent_joint_fixed_ = joint->getType() == JointModel::FIXED;
}

void LinkModel::setGeometry(const std::vector<shapes::ShapeConstPtr>& shapes,
                            const EigenSTL::vector_Isometry3d& origins)
{
  shapes_ = shapes;
  collision_origin_transform_ = origins;
  collision_origin_transform_is_identity_.resize(collision_origin_transform_.size());

  core::AABB aabb;

  for (std::size_t i = 0; i < shapes_.size(); ++i)
  {
    collision_origin_transform_is_identity_[i] =
        (collision_origin_transform_[i].rotation().isIdentity() &&
         collision_origin_transform_[i].translation().norm() < std::numeric_limits<double>::epsilon()) ?
            1 :
            0;
    Eigen::Isometry3d transform = collision_origin_transform_[i];

    if (shapes_[i]->type != shapes::MESH)
    {
      Eigen::Vector3d extents = shapes::computeShapeExtents(shapes_[i].get());
      aabb.extendWithTransformedBox(transform, extents);
    }
    else
    {
      // we cannot use shapes::computeShapeExtents() for meshes, since that method does not provide information about
      // the offset of the mesh origin
      const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shapes_[i].get());
      for (unsigned int j = 0; j < mesh->vertex_count; ++j)
      {
        aabb.extend(transform * Eigen::Map<Eigen::Vector3d>(&mesh->vertices[3 * j]));
      }
    }
  }

  centered_bounding_box_offset_ = aabb.center();
  if (shapes_.empty())
    shape_extents_.setZero();
  else
    shape_extents_ = aabb.sizes();
}

void LinkModel::setVisualMesh(const std::string& visual_mesh, const Eigen::Isometry3d& origin,
                              const Eigen::Vector3d& scale)
{
  visual_mesh_filename_ = visual_mesh;
  visual_mesh_origin_ = origin;
  visual_mesh_scale_ = scale;
}

}  // end of namespace core
}  // end of namespace moveit


  // Eigen Isometry3d
  template<>
  struct std::hash<Eigen::Isometry3d>
  {
    std::size_t operator()(const Eigen::Isometry3d& transform_matrix) const 
    {
        std::size_t h = 0;
        const Eigen::Transform<double, 3, Eigen::Isometry>::MatrixType matrix = transform_matrix.matrix();
        for (auto i = 0; i < 4; i++) 
        {
          for (auto j = 0; j < 4; j++) 
          {
            boost::hash_combine(h, std::hash<double>{}(matrix(i, j)));
          }
        }
        return h;
    }
  };

  // Eigen Vector3d
  template<>
  struct std::hash<Eigen::Vector3d>
  {
    std::size_t operator()(const Eigen::Vector3d& v) const 
    {
        std::size_t h = 0;
        for (auto i = 0; i < 3; i++) 
        {
          boost::hash_combine(h, std::hash<double>{}(v(i,0)));
        }
        return h;
    }
  };

  // ROS Shapes
  template<>
  struct std::hash<shapes::Shape>
  {
    const std::string LOGNAME = "link_model";
    virtual std::size_t operator()(const shapes::Shape &shape) const
    {
     // 1. I can make it pure virtual but then Plane/Octree needs to
     //    implement this. But Octee doesn't seem easy and since we
     //    don't use it in urdf, I don't feel like going with tihs
     //    route
     // 2. Throw exception
     // 3. Return some random num
    throw moveit::ConstructException("... we're using Shape (base) class implementation for hash");
    }
  };

  template<> 
  struct std::hash<shapes::Box>
  {
    std::size_t operator()(const shapes::Box& box) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, std::hash<std::string>{}(shapes::Box::STRING_NAME));
      // box size
      for (auto i = 0; i < 3; i++)
        boost::hash_combine(h, std::hash<double>{}(box.size[i]));
      return h;
    }
  };

  template<> 
  struct std::hash<shapes::Cylinder>
  {
    std::size_t operator()(const shapes::Cylinder& c) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, std::hash<std::string>{}(shapes::Cylinder::STRING_NAME));
      boost::hash_combine(h, std::hash<double>{}(c.length));
      boost::hash_combine(h, std::hash<double>{}(c.radius));
      return h;
    }
  };

  template<> 
  struct std::hash<shapes::Mesh>
  {
    std::size_t operator()(const shapes::Mesh& m) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, std::hash<std::string>{}(shapes::Mesh::STRING_NAME));
      boost::hash_combine(h, std::hash<unsigned int>{}(m.triangle_count));
      auto n = 3 * m.triangle_count;
      for(unsigned int i  = 0; i < n; i++) {
        boost::hash_combine(h, std::hash<unsigned int>{}(m.triangles[i]));
        boost::hash_combine(h, std::hash<unsigned int>{}(m.triangle_normals[i]));
      }
      boost::hash_combine(h, std::hash<unsigned int>{}(m.vertex_count));
      n = 3 * m.vertex_count;
      for(unsigned int i = 0; i < n; i++) {
        boost::hash_combine(h, std::hash<double>{}(m.vertices[i]));
        boost::hash_combine(h, std::hash<double>{}(m.vertex_normals[i]));
      }
      return h;
    }
  };

  template<> 
  struct std::hash<shapes::Sphere>
  {
    std::size_t operator()(const shapes::Sphere& s) const 
    {
      std::size_t h = 0;
      boost::hash_combine(h, std::hash<std::string>{}(shapes::Sphere::STRING_NAME));
      boost::hash_combine(h, std::hash<double>{}(s.radius));
      return h;
    }
  };

  template<> 
  struct std::hash<moveit::core::LinkModel>
  {
    std::size_t operator()(moveit::core::LinkModel const& link) const noexcept
    {
			std::size_t h =0;
      // use Link name_
			boost::hash_combine(h, std::hash<std::string>{}(link.getName()));
      boost::hash_combine(h, std::hash<moveit::core::JointModel>{}(*link.getParentJointModel()));
      boost::hash_combine(h, std::hash<moveit::core::LinkModel>{}(*link.getParentLinkModel()));
      boost::hash_combine(h, std::hash<bool>{}(link.parentJointIsFixed()));
      boost::hash_combine(h, std::hash<bool>{}(link.jointOriginTransformIsIdentity()));

      // rows = 3 and cols = 4 since it is Isometry3d
      boost::hash_combine(h, std::hash<Eigen::Isometry3d>{}(link.getJointOriginTransform()));

      auto collision_origin_transform = link.getCollisionOriginTransforms();
      for (auto& collision_isometry : collision_origin_transform) {
        boost::hash_combine(h, std::hash<Eigen::Isometry3d>{}(collision_isometry));
      }

      auto collision_origin_transforms_identity = link.areCollisionOriginTransformsIdentity();
      for (auto& identity : collision_origin_transforms_identity) {
        boost::hash_combine(h, std::hash<int>{}(identity));
      }

      auto associated_fixed_transforms = link.getAssociatedFixedTransforms();
      for (auto const& x: associated_fixed_transforms) {
          const Eigen::Isometry3d matrix = x.second;
          boost::hash_combine(h, std::hash<Eigen::Isometry3d>{}(matrix));
      }
    
      auto shapes = link.getShapes(); 
      for(auto const& x : shapes) {
        boost::hash_combine(h, std::hash<shapes::Shape>{}(*x));
      }
      boost::hash_combine(h, std::hash<Eigen::Vector3d>{}(link.getShapeExtentsAtOrigin()));
      boost::hash_combine(h, std::hash<Eigen::Vector3d>{}(link.getCenteredBoundingBoxOffset()));
      
      boost::hash_combine(h, std::hash<string>{}(link.getVisualMeshFilename()));

      boost::hash_combine(h, std::hash<Eigen::Isometry3d>{}(link.getVisualMeshOrigin()));

      boost::hash_combine(h, std::hash<Eigen::Vector3d>{}(link.getVisualMeshScale()));

      boost::hash_combine(h, std::hash<int>{}(link.getFirstCollisionBodyTransformIndex()));

      boost::hash_combine(h, std::hash<int>{}(link.getLinkIndex()));
      return h;

    }
  };