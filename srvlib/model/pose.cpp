#include <srvlib/model/pose.hpp>
#include <srvlib/srvlib.hpp>

using namespace srvlib;

Pose::operator glm::mat4() const {

  //cv::Vec3f euler = rotation_.EulerAngles();
  //glm::mat4 ret = glm::mat4::createRotation(ci::Vec3d(euler[0],euler[1],euler[2]));
  glm::quat q = rotation_;
  glm::mat4 ret = (glm::mat4)q;
  
  for (int i = 0; i < 3; i++){
    ret[3][i] = translation_[i];
  }
    
  return ret;

}

cv::Vec3f Pose::TransformPoint(const cv::Vec3f &point_in_world_coordinates) const{

  const glm::vec3 vec(point_in_world_coordinates[0], point_in_world_coordinates[1], point_in_world_coordinates[2]);
  const glm::vec3 ret = TransformPoint(vec);

  return cv::Vec3f(ret[0], ret[1], ret[2]);

}

cv::Vec3f Pose::InverseTransformPoint(const cv::Vec3f &point_in_model_coordinates) const {

  glm::mat4 pose_inv = *this;
  pose_inv = glm::inverse(pose_inv);

  const glm::vec3 vec(point_in_model_coordinates[0], point_in_model_coordinates[1], point_in_model_coordinates[2]);
  const glm::vec3 ret(pose_inv * unproject(vec));

  return cv::Vec3f(ret[0], ret[1], ret[2]);

}

glm::vec3 Pose::TransformPoint(const glm::vec3 &point_in_world_coordinates) const{

  glm::mat4 pose = *this;

  return project(pose * unproject(point_in_world_coordinates));

}

void Pose::SetPose(std::vector<float> &pose) {

  assert(pose.size() == 7);

  translation_[0] = pose[0];
  translation_[1] = pose[1];
  translation_[2] = pose[2];

  //rotation_ = sv::Quaternion(pose[3], pose[4], pose[5], pose[6]);
  //rotation_ = rotation_.Normalize();
  rotation_ = glm::quat(pose[3], pose[4], pose[5], pose[6]);
  rotation_ = glm::normalize(rotation_);

}



std::vector<float> Pose::GetPose() const{

  std::vector<float> ret;
  ret.push_back(translation_[0]);
  ret.push_back(translation_[1]);
  ret.push_back(translation_[2]);
  ret.push_back(rotation_.w);
  ret.push_back(rotation_.x);
  ret.push_back(rotation_.y);
  ret.push_back(rotation_.z);
  return ret;

}

Pose::Pose(const glm::mat4 &t) {

  translation_ = glm::vec3(t[3]);

  glm::mat3 r(t);
  
  //rotation_ = sv::Quaternion(rotation);
  rotation_ = glm::quat(r);

}

std::vector<glm::vec3> Pose::ComputeJacobian(const glm::vec3 &point_) const {

  glm::mat4 self_pose = *this;
  
  glm::vec3 point = project(glm::inverse(self_pose) * unproject(point_));

  std::vector<glm::vec3> data(7);

  //translation dofs
  data[0] = glm::vec3(1.0f, 0.0f, 0.0f);
  data[1] = glm::vec3(0.0f, 1.0f, 0.0f);
  data[2] = glm::vec3(0.0f, 0.0f, 1.0f);

  //rotation dofs - Qw
  data[3] = glm::vec3(
    (2.0f * (float)rotation_.y * point[2]) - (2.0f * (float)rotation_.z * point[1]),
    (2.0f * (float)rotation_.z * point[0]) - (2.0f * (float)rotation_.x * point[2]),
    (2.0f * (float)rotation_.x * point[1]) - (2.0f * (float)rotation_.y * point[0])
    );                                                                        
                                                                              
  // Qx                                                                       
  data[4] = glm::vec3(                                                        
    (2.0f * (float)rotation_.y * point[1]) + (2.0f * (float)rotation_.z * point[2]),
    (2.0f * (float)rotation_.y * point[0]) - (4.0f * (float)rotation_.x * point[1]) - (2.0f * (float)rotation_.w * point[2]),
    (2.0f * (float)rotation_.z * point[0]) + (2.0f * (float)rotation_.w * point[1]) - (4.0f * (float)rotation_.x * point[2])
    );                                                                                                                   
                                                                                                                         
  // Qy                                                                                                                  
  data[5] = glm::vec3(                                                                                                   
    (2.0f * (float)rotation_.x * point[1]) - (4.0f * (float)rotation_.y * point[0]) + (2.0f * (float)rotation_.w * point[2]),
    (2.0f * (float)rotation_.x * point[0]) + (2.0f * (float)rotation_.z * point[2]),                                 
    (2.0f * (float)rotation_.z * point[1]) - (2.0f * (float)rotation_.w * point[0]) - (4.0f * (float)rotation_.y * point[2])
    );                                                                                                                   
                                                                                                                         
  // Qz  ==> previously [1] was (2.0f * (float)rotation_.w * point[0]) - (4.0f * (float)rotation_.x * point[1]) + (2.0f * (float)rotation_.y * point[2]),                                                                                           
  data[6] = glm::vec3(                                                                                                   
    (2.0f * (float)rotation_.x * point[2]) - (2.0f * (float)rotation_.w * point[1]) - (4.0f * (float)rotation_.z * point[0]),
    (2.0f * (float)rotation_.w * point[0]) - (4.0f * (float)rotation_.z * point[1]) + (2.0f * (float)rotation_.y * point[2]),
    (2.0f * (float)rotation_.x * point[0]) + (2.0f * (float)rotation_.y * point[1])
    );
  
  return data;
}

void Pose::UpdatePose(const std::vector<float> &updates) {

  assert(updates.size() == GetNumDofs());

  for (int i = 0; i < 3; ++i)
    translation_[i] += updates[i];

  //w,x,y,z
  glm::quat rotation_update(updates[3], updates[4], updates[5], updates[6]);

  rotation_ = rotation_ + rotation_update;
  rotation_ = glm::normalize(rotation_);
}
