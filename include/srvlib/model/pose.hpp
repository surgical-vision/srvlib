#ifndef __POSE_HPP__
#define __POSE_HPP__

#include <cinder/Matrix44.h>
#include <cinder/Vector.h>
#include <cinder/Quaternion.h>
#include <opencv/cv.h>

namespace srvlib {

  /**
  * @class Pose
  * @brief Representation of pose for a rigid body.
  * Contains a SE3 transformation
  */
  class Pose {

  public:

    /**
    * Construct an empty pose. Rotation is set to identity and translation to zero vector.
    */
    Pose() {}

    /**
    * Default construction of pose from rotation and translation from base coordinate system.
    * @param[in] rotation The rotation as a quaternion.
    * @param[in] translation The translation as a 3-vector.
    */
    Pose(const glm::quat &rotation, const glm::vec3 &translation) : rotation_(rotation), translation_(translation) {}

    /**
    * Default construction of pose an SE3 type matrix.
    * @param[in] transform The transform in a 4x4 matrix.
    */
    Pose(const glm::mat4 &transform);

    /**
    * Compute the rigid body jacobian for this pose for a particular 3D point.
    * @param[in] point The 3D point to compute the jacobian from.
    * @return The jacobian as a vector of partial derivates. Each 3-vector is \f$ \big(\frac{\partial X}{\partial \lambda_{n}},\frac{\partial Y}{\partial \lambda_{n}},\frac{\partial Z}{\partial \lambda_{n}}\big)\f$ (allows more abstract handling).
    */
    std::vector<glm::vec3> ComputeJacobian(const glm::vec3 &point) const;

    /**
    * Set the pose updates.
    * @param[in] updates The vector of updates for each degree of freedom.
    */
    void UpdatePose(const std::vector<float> &updates);

    std::vector<float> GetPose() const;

    glm::quat GetRotation() const { return rotation_; }
    glm::vec3 GetTranslation() const { return translation_; }

    void SetPose(std::vector<float> &pose);
    void SetPose(const glm::mat4 &pose);
    /**
    * Get the number of degrees of freedom associated with the pose.
    * @return The number of degrees of freedom.
    */
    std::size_t GetNumDofs() const { return 7; }

    /**
    * Useful for GL transforms when we need to draw the model at some pose estimate.
    */
    operator glm::mat4() const;

    /**
    * Transform a point into the pose coordinates.
    * @param[in] point_in_world_coordinates The point it the world coordinates.
    * @return The point the pose coordinates.
    */
    glm::vec3 TransformPoint(const glm::vec3 &point_in_world_coordinates) const;

    /**
    * Transform a point into the pose coordinates.
    * @param[in] point_in_world_coordinates The point it the world coordinates.
    * @return The point the pose coordinates.
    */
    cv::Vec3f TransformPoint(const cv::Vec3f &point_in_world_coordinates) const;

    /**
    * Transform a point into the world coordinates from pose coordinates.
    * @param[in] point_in_model_coordinates The point it the pose/model coordinates.
    * @return The point the world coordinates.
    */
    cv::Vec3f InverseTransformPoint(const cv::Vec3f &point_in_model_coordinates) const;

    /**
    * Write pose to stream. Useful for debugging etc.
    * @param[in] os The stream to write to.
    * @param[in] pose The pose to write to the stream.
    * @return The stream reference for operator concatenation.
    */
    friend std::ostream &operator<<(std::ostream &os, const Pose &pose);

  protected:

    glm::vec3 translation_; /**< The translation vector. */
    glm::quat rotation_; /**< The rotation matrix. */

  };

  inline std::ostream &operator<<(std::ostream &os, const Pose &p){
    os << "[" << p.rotation_ << ", " << p.translation_[0] << ", " << p.translation_[1] << ", " << p.translation_[2] << "]";
    return os;
  }

}


#endif
