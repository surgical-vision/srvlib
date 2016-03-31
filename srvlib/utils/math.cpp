#include <srvlib/utils/math.hpp>


glm::vec3 srvlib::math::GetZYXEulersFromQuaternion(const glm::quat &quaternion) {

  glm::vec3 angles;

  angles[0] = atan2(2 * (quaternion.z*quaternion.w - quaternion.x*quaternion.y), (quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z));
  angles[1] = asin(2 * (quaternion.x*quaternion.z + quaternion.y*quaternion.w));
  angles[2] = atan2(2 * (quaternion.x*quaternion.w - quaternion.y*quaternion.z),
    (quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z));

  return angles;

}

glm::vec3 srvlib::math::GetXZYEulersFromQuaternion(const glm::quat &quaternion)  {

  glm::vec3 angles;

  angles[0] = atan2(2 * (quaternion.x*quaternion.z - quaternion.x*quaternion.y), (quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z));
  angles[1] = asin(2 * (quaternion.w*quaternion.z - quaternion.y*quaternion.x));
  angles[2] = atan2(2 * (quaternion.x*quaternion.w + quaternion.y*quaternion.z), (quaternion.w * quaternion.w - quaternion.x * quaternion.x + quaternion.y * quaternion.y - quaternion.z * quaternion.z));

  return angles;
}

glm::vec3 srvlib::math::GetXYZEulersFromQuaternion(const glm::quat &quaternion) {

  glm::vec3 angles;

  angles[0] = atan2(2 * (quaternion.x*quaternion.w + quaternion.z*quaternion.y), (quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z));
  angles[1] = asin(2 * (quaternion.y*quaternion.w - quaternion.x*quaternion.z));
  angles[2] = atan2(2 * (quaternion.x*quaternion.y + quaternion.z*quaternion.w), (quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z));
  
  return angles;
  
}

glm::mat3 srvlib::math::MatrixFromIntrinsicEulers(float xRotation, float yRotation, float zRotation, const std::string &order) {

  float cosx = cos(xRotation);
  float cosy = cos(yRotation);
  float cosz = cos(zRotation);
  float sinx = sin(xRotation);
  float siny = sin(yRotation);
  float sinz = sin(zRotation);

  glm::mat3 xRotationMatrix;
  glm::mat3 yRotationMatrix;
  glm::mat3 zRotationMatrix;

  xRotationMatrix[1][1] = xRotationMatrix[2][2] = cosx;
  xRotationMatrix[2][1] = -sinx;
  xRotationMatrix[1][2] = sinx;

  yRotationMatrix[0][0] = yRotationMatrix[2][2] = cosy;
  yRotationMatrix[2][0] = siny;
  yRotationMatrix[0][2] = -siny;

  zRotationMatrix[0][0] = zRotationMatrix[1][1] = cosz;
  zRotationMatrix[1][0] = -sinz;
  zRotationMatrix[0][1] = sinz;

  glm::mat3 r;

  //zyx
  if (order == "zyx")
    r = xRotationMatrix * yRotationMatrix * zRotationMatrix;
  else if (order == "xyz")
    r = zRotationMatrix * yRotationMatrix * xRotationMatrix;
  else if (order == "xzy")
    r = yRotationMatrix * zRotationMatrix * xRotationMatrix;
  else
    throw std::runtime_error("");

  return r;

}


void srvlib::math::set_translate(glm::mat4 &m, const glm::vec3 &t) {

  for (int i = 0; i < 3; ++i){
    m[3][i] = t[i];
  }
  
}