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

#include <srvlib/view/camera.hpp>

using namespace srvlib;

void Camera::Setup(const cv::Mat camera_matrix, const cv::Mat distortion_params, const size_t image_width, const size_t image_height, const size_t near_clip_distance, const size_t far_clip_distance){

  image_width_ = image_width;
  image_height_ = image_height;
  near_clip_distance_ = near_clip_distance;
  far_clip_distance_ = far_clip_distance;

  //load opencv camera calibration parameters
  camera_matrix_ = camera_matrix.clone();
  distortion_params_ = distortion_params.clone();

  gl_projection_matrix_ = glm::mat4();
  gl_projection_matrix_[0][0] = (float)camera_matrix_.at<double>(0, 0);
  gl_projection_matrix_[1][1] = (float)camera_matrix_.at<double>(1, 1);
  gl_projection_matrix_[2][0] = (float)-camera_matrix_.at<double>(0, 2);
  gl_projection_matrix_[2][1] = (float)-camera_matrix_.at<double>(1, 2);
  gl_projection_matrix_[2][2] = (float)(near_clip_distance + far_clip_distance);
  gl_projection_matrix_[3][2] = (float)(near_clip_distance * far_clip_distance);
  gl_projection_matrix_[2][3] = -1;

  is_setup_ = true;

}

void Camera::makeCurrentCamera() const {

  ci::CameraOrtho o;
  o.setOrtho(0.0f, (float)image_width_, 0.0f, (float)image_height_, (float)near_clip_distance_, (float)far_clip_distance_);

  ci::gl::setProjectionMatrix(o.getProjectionMatrix());
  ci::gl::multProjectionMatrix(gl_projection_matrix_);

}

glm::vec2 Camera::ProjectVertex(const glm::vec3 point_in_camera_coordinates) const {

  std::vector<cv::Point2d> projected_point;
  static cv::Mat rot = cv::Mat::eye(3, 3, CV_64FC1);
  static cv::Mat tran = cv::Mat::zeros(3, 1, CV_64FC1);

  cv::projectPoints(std::vector<cv::Point3d>(1, cv::Point3f(point_in_camera_coordinates[0], point_in_camera_coordinates[1], point_in_camera_coordinates[2])), rot, tran, camera_matrix_, distortion_params_, projected_point);
  if (projected_point.size() != 1) throw(std::runtime_error("Error, projected points size != 1.\n"));

  cv::Point2d &p = projected_point.front();
  return glm::vec2(p.x, p.y);

}

glm::ivec2 Camera::ProjectVertexToPixel(const glm::vec3 point_in_camera_coordinates) const{

  glm::vec2 r = ProjectVertex(point_in_camera_coordinates);
  return glm::ivec2((int)round(r.x), (int)round(r.y));

}
  

void StereoCamera::Setup(const std::string &calibration_filename, const int near_clip_distance, const int far_clip_distance){

  if (!boost::filesystem::exists(boost::filesystem::path(calibration_filename)))
    throw(std::runtime_error("Error, could not find camera calibration file: " + calibration_filename + "\n"));

  cv::FileStorage fs;

  try{

    cv::Mat l_intrinsic, l_distortion;
    cv::Mat r_intrinsic, r_distortion;
    fs.open(calibration_filename, cv::FileStorage::READ);

    fs["Left_Camera_Matrix"] >> l_intrinsic;
    fs["Left_Distortion_Coefficients"] >> l_distortion;
    fs["Right_Camera_Matrix"] >> r_intrinsic;
    fs["Right_Distortion_Coefficients"] >> r_distortion;

    cv::Mat rotation(3, 3, CV_64FC1), translation(3, 1, CV_64FC1);
    fs["Extrinsic_Camera_Rotation"] >> rotation;
    fs["Extrinsic_Camera_Translation"] >> translation;

    cv::Mat image_size;
    fs["Image_Dimensions"] >> image_size;
    size_t image_width = image_size.at<int>(0);
    size_t image_height = image_size.at<int>(1);

    convertBouguetToGLCoordinates(l_intrinsic, r_intrinsic, rotation, translation, image_width, image_height);

    left_eye_.Setup(l_intrinsic, l_distortion, image_width, image_height, near_clip_distance, far_clip_distance);
    right_eye_.Setup(r_intrinsic, r_distortion, image_width, image_height, near_clip_distance, far_clip_distance);

    for (int r = 0; r<rotation.rows; r++){
      for (int c = 0; c<rotation.cols; c++){
        extrinsic_rotation_[c][r] = (float)rotation.at<double>(r, c);
      }
      extrinsic_translation_[r] = (float)translation.at<double>(r, 0);
    }

  }
  catch (cv::Exception& e){

    std::cerr << "Error while reading from camara calibration file.\n" << e.msg << "\n";
    throw std::runtime_error("");

  }

}

void StereoCamera::convertBouguetToGLCoordinates(cv::Mat &left_camera_matrix, cv::Mat &right_camera_matrix, cv::Mat &extrinsic_rotation, cv::Mat &extrinsic_translation, const size_t image_width, const size_t image_height){

  //first flip the principal points
  left_camera_matrix.at<double>(1, 2) = image_height - left_camera_matrix.at<double>(1, 2);
  right_camera_matrix.at<double>(1, 2) = image_height - right_camera_matrix.at<double>(1, 2);

  extrinsic_rotation = extrinsic_rotation.inv();
  extrinsic_translation = extrinsic_translation * -1;

}

void StereoCamera::setupLeftCamera(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose){


  glGetIntegerv(GL_VIEWPORT, viewport_cache_);
  glViewport(0, 0, (int)left_eye_.getImageWidth(), (int)left_eye_.getImageHeight());
  moveEyeToLeftCam(shader, current_camera_pose);

}


void StereoCamera::setupRightCamera(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose){

  glViewport(0, 0, (int)right_eye_.getImageWidth(), (int)right_eye_.getImageHeight());
  moveEyeToRightCam(shader, current_camera_pose);

}


void StereoCamera::unsetCameras(){

  glViewport(viewport_cache_[0], viewport_cache_[1], viewport_cache_[2], viewport_cache_[3]);

}

void StereoCamera::makeLeftEyeCurrent(){

  left_eye_.makeCurrentCamera();

}

void StereoCamera::makeRightEyeCurrent(){

  right_eye_.makeCurrentCamera();

}

void StereoCamera::TurnOnLight(){

  throw std::runtime_error("StereoCamera::TurnOnLight()");

}

void StereoCamera::TurnOffLight(){

  throw std::runtime_error("Disable lighting.\n");

}

void StereoCamera::moveEyeToLeftCam(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose){

  glm::vec3 eye_point(0, 0, 0);
  glm::vec3 view_direction(0, 0, 1);
  glm::vec3 world_up(0, -1, 0);

  ci::CameraPersp *camP = GetLeftCamera().GetPerspectiveCamera();
  camP->setEyePoint(eye_point);
  camP->setViewDirection(view_direction);
  camP->setWorldUp(world_up);

  left_eye_.makeCurrentCamera();

  glm::vec3 mvLightPos = glm::vec3(ci::gl::getModelView() * glm::vec4(0.0, 0.0, 0.0, 1.0f));
  glm::mat4 shadowMatrix = camP->getProjectionMatrix() * camP->getViewMatrix();

  shader->uniform("tex0", 0);
  shader->uniform("uShadowMap", 0);
  shader->uniform("uLightPos", mvLightPos);
  shader->uniform("uShadowMatrix", shadowMatrix);

  ci::gl::setMatrices(*camP);

  ci::gl::multModelMatrix(glm::inverse(current_camera_pose));


}


void StereoCamera::moveEyeToRightCam(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose){

  glm::vec3 eye_point(0, 0, 0);
  glm::vec3 view_direction(0, 0, 1);
  glm::vec3 world_up(0, -1, 0);

  ci::CameraPersp *camP = GetRightCamera().GetPerspectiveCamera();
  camP->setEyePoint(extrinsic_translation_);
  camP->setViewDirection(extrinsic_rotation_ * view_direction);
  camP->setWorldUp(extrinsic_rotation_ * world_up);
  
  glm::vec3 mvLightPos = glm::vec3(ci::gl::getModelView() * glm::vec4(0.0, 0.0, 0.0, 1.0f));
  glm::mat4 shadowMatrix = camP->getProjectionMatrix() * camP->getViewMatrix();

  shader->uniform("tex0", 0);
  shader->uniform("uShadowMap", 0);
  shader->uniform("uLightPos", mvLightPos);
  shader->uniform("uShadowMatrix", shadowMatrix);

  ci::gl::setMatrices(*camP);

  ci::gl::multModelMatrix(glm::inverse(current_camera_pose));

  right_eye_.makeCurrentCamera();

}