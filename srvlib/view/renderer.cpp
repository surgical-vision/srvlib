#include <srvlib/view/renderer.hpp>
#include <cinder/gl/gl.h>

#include <cinder/app/App.h>

using namespace srvlib;
using namespace srvlib::renderer;

void srvlib::renderer::DrawModel(std::shared_ptr<BaseModel> model, std::shared_ptr<StereoCamera> camera, const bool is_left, const glm::mat4 &camera_pose, ci::gl::GlslProgRef shader){

  ci::gl::enableDepthWrite();
  ci::gl::enableDepthRead();

  ci::gl::pushMatrices();

  shader->bind();

  if (is_left)
    camera->setupLeftCamera(shader, camera_pose);
  else
    camera->setupRightCamera(shader, camera_pose);

  model->Draw();

  camera->unsetCameras();
  ci::gl::popMatrices();

}

void srvlib::renderer::DrawModel(ci::gl::VboMeshRef model, std::shared_ptr<StereoCamera> camera, const bool is_left, const glm::mat4 &camera_pose, ci::gl::GlslProgRef shader){
  
  ci::gl::enableDepthWrite();
  ci::gl::enableDepthRead();

  ci::gl::pushMatrices();

  shader->bind();

  if (is_left)
    camera->setupLeftCamera(shader, camera_pose);
  else
    camera->setupRightCamera(shader, camera_pose);

  ci::gl::draw(model);

  camera->unsetCameras();
  ci::gl::popMatrices();

}

void srvlib::renderer::DrawTexture(ci::gl::Texture2dRef image, glm::ivec2 size, ci::gl::GlslProgRef shader){
  
  ci::gl::disableDepthRead();
  ci::gl::disableDepthWrite();

  ci::gl::pushMatrices();

  ci::CameraOrtho o;
  o.setOrtho(0.0f, (float)size[0], 0.0f, (float)size[1], (float)0, (float)1);

  ci::gl::setProjectionMatrix(o.getProjectionMatrix());

  ci::gl::setModelMatrix(glm::mat4());
  ci::gl::setViewMatrix(glm::mat4());

  image->bind();

  shader->bind();
  shader->uniform("tex0", 0);

  ci::Rectf bounds(0, 0, size[0], size[1]);
  ci::gl::drawSolidRect(bounds, glm::vec2(0, 0), glm::vec2(1, 1));

  image->unbind();

  ci::gl::popMatrices();

  ci::gl::enableDepthRead();
  ci::gl::enableDepthWrite();
}

void srvlib::renderer::DrawTexture(ci::gl::Texture2dRef image, glm::ivec2 size){

  ci::gl::disableDepthRead();
  ci::gl::disableDepthWrite();

  ci::gl::pushMatrices();

  ci::CameraOrtho o;
  o.setOrtho(0.0f, (float)size[0], 0.0f, (float)size[1], (float)0, (float)1);

  ci::gl::setProjectionMatrix(o.getProjectionMatrix());

  ci::gl::setModelMatrix(glm::mat4());
  ci::gl::setViewMatrix(glm::mat4());

  ci::gl::draw(image);

  ci::gl::popMatrices();

  ci::gl::enableDepthRead();
  ci::gl::enableDepthWrite();


}
