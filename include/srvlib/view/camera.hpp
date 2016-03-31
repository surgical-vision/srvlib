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

#include <opencv2/opencv.hpp>
#include <string>
#include <cinder/gl/gl.h>
#include <cinder/CameraUi.h>

namespace srvlib {

  /**
  * @class Camera
  * @brief Simple OpenGL friendly camera.
  * A simple class to handle drawing with a calibrated camera in an OpenGL environment. As we are working in surgical vision there is a light attached to each camera. Although not super realistic, it's easiest thing to do without calibrating the light position.
  */

  class Camera {

  public:

    /**
    * Default camera constructor. Sets an ID for the light and initilizes it as a directional light source.
    * @param[in] light_id The light ID.
    */
    explicit Camera(const int light_id) : persp_camera_(nullptr), is_setup_(false), light_id_(light_id) {}

    /**
    * Setup a camera from the standard calibration parameters.
    * @param[in] camera_matrix The camera matrix containing focal length and principal point.
    * @param[in] distortion_params The zhang polynomial model for camera distortion.
    * @param[in] image_width The x image resolution of the camera.
    * @param[in] image_height The y image resolution of the camera.
    * @param[in] near_clip The OpenGL near clip plane.
    * @param[in] far_clip The OpenGL far clip plane.
    */
    void Setup(const cv::Mat camera_matrix, const cv::Mat distortion_params, const size_t image_width, const size_t image_height, const size_t near_clip_distance, const size_t far_clip_distance);

    /**
    * Set the GL_PROJECTION matrix so that this camera is the one we are using. 
    */
    void makeCurrentCamera() const ;

    /**
    * Quick accessor for the image width.
    * @return The image width.
    */
    size_t getImageWidth() const { return image_width_; }
    
    /**
    * Quick accessor for the image height.
    * @return The image height.
    */    
    size_t getImageHeight() const { return image_height_; }

    /**
    * Switch on the light source.
    */
    void TurnOnLight();
    
    /**
    * Switch off the light source.
    */
    void TurnOffLight();

    ci::CameraPersp *GetPerspectiveCamera() { if (persp_camera_ == nullptr) persp_camera_ = new ci::CameraPersp(); return persp_camera_; }


    glm::vec2 ProjectVertex(const glm::vec3 point_in_camera_coordinates) const;
    glm::ivec2 ProjectVertexToPixel(const glm::vec3 point_in_camera_coordinates) const;

  protected:

    ci::CameraPersp *persp_camera_;

    cv::Mat camera_matrix_; /**< The camera calibration matrix. */
    glm::mat4 gl_projection_matrix_; /**< The GL_PROJECTIONMATRIX for this camera calibration. Ignores distortion. */
    cv::Mat distortion_params_; /**< The camera distortion parameters. */

    size_t image_width_; /**< The x resolution of the camera image. */
    size_t image_height_; /**< The y resolution of the camera image. */
    size_t near_clip_distance_; /**< The near clip plane for OpenGL. */
    size_t far_clip_distance_; /**< The far clip plane for OpenGL.  */

    bool is_setup_; /**< Flag for whether the camera calibration is loaded. */
     
    size_t light_id_; 

  };

  /**
  * @class StereoCamera
  * @brief Simple OpenGL friendly stereo camera.
  * A simple class to wrap 2 Camera objects in a single StereoCamera.
  */
  class StereoCamera {

  public:

    /**
    * Default constructor creating a left and right eye.
    */
    StereoCamera() : left_eye_(0), right_eye_(1) {}


    /**
    * Setup a camera from the standard calibration parameters using an OpenCV XML file.
    * @param[in] calibration_file An OpenCV calibration file.
    * @param[in] near_clip The OpenGL near clip plane.
    * @param[in] far_clip The OpenGL far clip plane.
    */
    void Setup(const std::string &calibration_file, const int near_clip_distance, const int far_clip_distance); 

    /**
    * Move the GL_MODELVIEW to the left camera position and setup the GL_VIEWPORT.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void setupLeftCamera(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose);

    /**
    * Move the GL_MODELVIEW to the right camera position and setup the GL_VIEWPORT.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void setupRightCamera(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose);

    /**
    * Set the GL_PROJECTION matrix so the left camera is the one we are rendering with.
    */
    void makeLeftEyeCurrent();
    
    /**
    * Set the GL_PROJECTION matrix so that the right camera is the one we are rendering with.
    */
    void makeRightEyeCurrent();

    /**
    * Reset the viewport.
    */
    void unsetCameras();

    /**
    * Accessor to get the extrinsic translation between the cameras.
    * @return the translation between the camera coordinates.
    */
    glm::vec3 getExtrinsicTranslation() const { return extrinsic_translation_; }
    
    /**
    * Accessor to get the extrinsic rotation between the cameras.
    * @return the rotation between the camera coordinates.
    */
    glm::mat3 getExtrinsicRotation() const { return extrinsic_rotation_; }

    /**
    * Switch on the left eye's light.
    */
    void TurnOnLight();
    
    /**
    * Switch off the left eye's light.
    */
    void TurnOffLight();

    /**
    * Accessor to get the left eye of the rig.
    * @return the left camera.
    */
    const Camera &GetLeftCamera() const { return left_eye_; }
    Camera &GetLeftCamera() { return left_eye_; }

    /**
    * Accessor to get the right eye of the rig.
    * @return the right camera.
    */
    const Camera &GetRightCamera() const { return right_eye_; }
    Camera &GetRightCamera() { return right_eye_; }

  protected:

    /**
    * Move the GL_MODELVIEW to correspond to the left camera and also move its light too.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void moveEyeToLeftCam(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose);

    /**
    * Move the GL_MODELVIEW to correspond to the right camera and also move its light too.
    * @param[in] cam A Cinder GL 'MayaCam' which is used to wrap up the data about this camera.
    * @param[in] current_camera_pose The world coordinates of the stereo camera eye.
    */
    void moveEyeToRightCam(ci::gl::GlslProgRef shader, const glm::mat4 &current_camera_pose);

    /**
    * Modifies a Bouguet camera calibration to make it compatible with OpenGL as they use different coordinate systems.
    * @param[in,out] left_camera_matrix The camera matrix of the left camera calibrated using J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in,out] right_camera_matrix The camera matrix of the right camera calibrated using J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in,out] extrinsic_rotation The extrinsic rotation computed by J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in,out] extrinsic_rotation The extrinsic translation computed by J.Y. Bouguet's toolbox. After function call it is set up for use in OpenGL.
    * @param[in] image_width Image width needed to transform the principal points.
    * @param[in] image_height Image height needed to transform the principal points.
    */
    void convertBouguetToGLCoordinates(cv::Mat &left_camera_matrix, cv::Mat &right_camera_matrix, cv::Mat &extrinsic_rotation, cv::Mat &extrinsic_translation, const size_t image_width, const size_t image_height);

    Camera left_eye_; /**< The camera corresponding to the stereo rig's left eye. */
    Camera right_eye_; /**< The camera corresponding to the stereo rig's right eye. */

    GLint viewport_cache_[4]; /**< Cache of the viewport (when we change it for the eyes so it's not lost). */

    glm::mat3 extrinsic_rotation_; /**< Rotation between the eye's coordinates system. If it's in Bouguet format this is rotation matrix which transforms points in left eye coordinate to right eye coordiantes. If GL then it's transformation that transforms coordinates system from left to right (i.e. the inverse of the Bouguet one). */
    glm::vec3 extrinsic_translation_; /**< Translation between the eye's coordinates system. If it's in Bouguet format this is translation vector which transforms points in left eye coordinate to right eye coordiantes. If GL then it's transformation that transforms coordinates system from left to right (i.e. the inverse of the Bouguet one). * */

  };


}