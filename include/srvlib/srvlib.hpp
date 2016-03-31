#pragma once

#include <glm/matrix.hpp>

namespace srvlib {

  inline glm::vec2 project(const glm::vec3 &v){

    return glm::vec2(v[0] / v[2], v[1] / v[2]);

  }

  inline glm::vec3 project(const glm::vec4 &v){

    return glm::vec3(v[0] / v[3], v[1] / v[3], v[2]/v[3]);

  }

  inline glm::vec3 unproject(const glm::vec2 &v){

    return glm::vec3(v[0], v[1], 1.0f);

  }

  inline glm::vec4 unproject(const glm::vec3 &v){

    return glm::vec4(v[0], v[1], v[2], 1.0f);

  }

}