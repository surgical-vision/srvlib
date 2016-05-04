#pragma once

#include <srvlib/model/model.hpp>
#include <srvlib/view/camera.hpp>

namespace srvlib {

  namespace renderer{

    void DrawModel(std::shared_ptr<Model> model, std::shared_ptr<StereoCamera> camera, const bool is_left, const glm::mat4 &camera_pose, ci::gl::GlslProgRef shader);
    void DrawModel(ci::gl::VboMeshRef model, const glm::mat4 &model_transform, std::shared_ptr<StereoCamera> camera, const bool is_left, const glm::mat4 &camera_pose, ci::gl::GlslProgRef shader);
    void DrawTexture(ci::gl::Texture2dRef model, const glm::ivec2 &size);
    void DrawTexture(ci::gl::Texture2dRef image, const glm::ivec2 &eye_size, const glm::ivec2 &draw_size, ci::gl::GlslProgRef shader);

  }

}

