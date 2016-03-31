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

namespace srvlib {

  /**
  * @class BaseModel
  * @brief An abstract class to represent 3D models. 
  * This class specifies the interface for loading models (which may be articulated) from configuration files, 
  * setting their poses and drawing them.
  */
  class BaseModel {

  public:

    /**
    * Draw the model at the current estimate of pose. Assumes that an OpenGL context is available for the active thread.
    */
    virtual void Draw() const = 0;

    /**
    * Load the data for the model from a config file.
    * @param[in] datafile_path The full path to the configuration file.
    */
    virtual void LoadData(const std::string &datafile_path) = 0;

    /**
    * Get the current set of transforms.
    * @return 
    */
    virtual std::vector<glm::mat4> GetTransformSet() const = 0;
    virtual void SetTransformSet(const std::vector<glm::mat4> &transforms) = 0;

  protected:
    
    struct RenderData {
      ci::TriMesh model_; /**< The 3D mesh that the model represents. */
      ci::gl::VboMeshRef	vbo_; /**< VBO to store the model for faster drawing. */
      ci::gl::TextureRef texture_; /**< The texture for the model. */
      glm::mat4 transform_; /**< The transform from world coordinates to the model coordinate system. */
    };


    /**
    * Draw a single RenderData model
    * @param[in] rd The model to draw.
    */
    void InternalDraw(const RenderData &rd, const float inc=0) const;

    ci::JsonTree OpenFile(const std::string &datafile_path) const;
    
    void LoadComponent(const ci::JsonTree &tree, RenderData &target, const std::string &root_dir);

  };

  class Model : public BaseModel {
  
  public:

    virtual void Draw() const;
    virtual void LoadData(const std::string &datafile_path);

    virtual std::vector<glm::mat4> GetTransformSet() const;
    virtual void SetTransformSet(const std::vector<glm::mat4> &transforms);

    RenderData &Body() { return body_; }
    const RenderData &Body() const { return body_; }

  protected:

    RenderData body_;

  };
  
}


















