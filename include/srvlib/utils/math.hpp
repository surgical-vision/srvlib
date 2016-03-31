#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/matrix.hpp>
#include <string>

namespace srvlib {

  namespace math {

    glm::vec3 GetZYXEulersFromQuaternion(const glm::quat &r); 
    glm::vec3 GetXZYEulersFromQuaternion(const glm::quat &r);
    glm::vec3 GetXYZEulersFromQuaternion(const glm::quat &r);

    glm::mat3 MatrixFromIntrinsicEulers(const float x_rotation, const float y_rotation, const float z_rotation, const std::string &order);

    void set_translate(glm::mat4 &m, const glm::vec3 &t);





  }
}