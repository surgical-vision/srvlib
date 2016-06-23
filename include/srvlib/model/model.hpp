#pragma once

/**

srvlib - A robotics visualizer specialized for the da Vinci robotic system.
Copyright (C) 2014 Max Allan

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

**/

#include <cinder/ObjLoader.h>
#include <cinder/TriMesh.h>
#include <cinder/gl/Vbo.h>
#include <cinder/gl/VboMesh.h>
#include <cinder/gl/Texture.h>
#include <cinder/Json.h>

#include "node.hpp"
#include "pose.hpp"

namespace srvlib {

  /**
  * @class Model
  * @brief An class to represent 3D models. 
  * This class specifies the interface for loading models (which may be articulated) from configuration files, 
  * setting their poses and drawing them.
  */
  class Model {

  public:

    /**
    * Draw the model at the current estimate of pose. Assumes that an OpenGL context is available for the active thread.
    */
    virtual void Draw();

    /**
    * Load the data for the model from a config file.
    * @param[in] datafile_path The full path to the configuration file.
    */
    virtual void LoadData(const std::string &datafile_path);

    void SetBasePose(const glm::mat4 &pose) { base_pose_ = pose; }

    Pose GetPose() const;

  protected:
    
    Node::Ptr internal_model_;
    Pose base_pose_; //actual pose

    /**
    * Draw a single RenderData model
    * @param[in] rd The model to draw.
    */
    void InternalDraw(Node::Ptr rd, const float inc = 0);

    ci::JsonTree OpenFile(const std::string &datafile_path) const;
    
    void LoadComponent(const ci::JsonTree &tree, Node::Ptr target, const std::string &root_dir);

  };
  
}


















