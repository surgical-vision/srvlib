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

#include <cinder/app/App.h>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/matrix_interpolation.hpp>

#include <srvlib/srvlib.hpp>
#include <srvlib/io/pose/pose_grabber.hpp>
#include <srvlib/utils/math.hpp>

using namespace srvlib;

inline void clean_string(std::string &str, const std::vector<char> &to_remove){
  for (auto &ch : to_remove){
    str.erase(std::remove(str.begin(), str.end(), ch), str.end());
  }
}

size_t BasePoseGrabber::grabber_num_id_ = 0;


std::string BasePoseGrabber::WriteSE3ToString(const glm::mat4 &mat){

  std::stringstream ss;

  for (int r = 0; r < 4; ++r){
    ss << "| ";
    for (int c = 0; c < 4; ++c){
      ss << mat[c][r] << " ";
    }
    ss << "|\n";
  }

  return ss.str();

}

BasePoseGrabber::BasePoseGrabber(const std::string &output_dir) : do_draw_(false) , save_dir_(output_dir) {

  std::stringstream ss;
  ss << "Pose grabber " << grabber_num_id_;
  param_modifier_ = ci::params::InterfaceGl::create(ci::app::getWindow(), ss.str(), ci::app::toPixels(glm::ivec2(50, 50)));
  param_modifier_->hide();  

  grabber_num_id_++;

}

glm::mat4 SE3DaVinciPoseGrabber::RemoveOutOfPlaneRotation(const glm::mat4 &pose) const {

  glm::quat rotation = glm::quat_cast(pose);
  glm::vec3 eulers = GetXZYEulersFromQuaternion(rotation);
  
  eulers[1] = 0;
  glm::mat4 m = MatrixFromIntrinsicEulers(eulers[0], 0, 0, "xzy");
  srvlib::math::set_translate(m, glm::vec3(pose[3]));

  return m;
}

void SE3DaVinciPoseGrabber::GetSubWindowCoordinates(const srvlib::Camera &camera, std::array<glm::ivec2, 4> &rectangle, cv::Mat &affine_transform) {

  const glm::mat4 pose = GetPose();
  glm::quat rotation = glm::quat_cast(pose);
  glm::vec3 eulers = GetXZYEulersFromQuaternion(rotation);
  const float x_rotation = eulers[0];

  const glm::vec2 center_of_mass = camera.ProjectVertexToPixel(glm::vec3(pose * glm::vec4(0, 0, 0, 1)));
  const glm::vec2 angle_of_shaft = camera.ProjectVertexToPixel(glm::vec3(pose * glm::vec4(0, 0, 10, 1)));

  glm::vec2 local_vertical_axis = angle_of_shaft - center_of_mass; 
  glm::normalize(local_vertical_axis);
  glm::vec2 local_horizontal_axis = glm::vec2(local_vertical_axis[1], -local_vertical_axis[0]);

#ifdef _WIN32
  float distance = std::sqrtf((center_of_mass.x - angle_of_shaft.x)*(center_of_mass.x - angle_of_shaft.x) + (center_of_mass.y - angle_of_shaft.y)*(center_of_mass.y - angle_of_shaft.y));
#else
  float distance = std::sqrt((float)((center_of_mass.x - angle_of_shaft.x)*(center_of_mass.x - angle_of_shaft.x) + (center_of_mass.y - angle_of_shaft.y)*(center_of_mass.y - angle_of_shaft.y)));
#endif


  glm::vec2 top_left = center_of_mass + (2.4f * distance * local_vertical_axis) + (2 * distance * local_horizontal_axis);
  glm::vec2 top_right = center_of_mass + (2.4f * distance * local_vertical_axis) - (2 * distance * local_horizontal_axis);

  glm::vec2 bottom_left = center_of_mass + (2.0f * distance * local_horizontal_axis);
  glm::vec2 bottom_right = center_of_mass - (2.0f * distance * local_horizontal_axis);

  rectangle[0] = top_left;
  rectangle[1] = top_right;
  rectangle[2] = bottom_right;
  rectangle[3] = bottom_left;

  affine_transform = cv::Mat::eye(cv::Size(3, 2), CV_32FC1);
  float angle = acos(std::abs(local_horizontal_axis[1])); //minus as we're going back from subwindow to window coords
  affine_transform.at<float>(0, 0) = affine_transform.at<float>(0, 0) = cos(angle);
  affine_transform.at<float>(0, 1) = -sin(angle);
  affine_transform.at<float>(1, 0) = sin(angle);
  affine_transform.at<float>(0, 2) = bottom_left[0];
  affine_transform.at<float>(1, 2) = bottom_left[1];

}

void BasePoseGrabber::convertFromBouguetPose(const glm::mat4 &in_pose, glm::mat4 &out_pose){

  //out_pose.setToIdentity();
  out_pose = glm::mat4();

  glm::vec3 translation = project(in_pose[3]);
  translation[1] *= -1;
  translation[2] *= -1;
  srvlib::math::set_translate(out_pose, translation);

  glm::mat4 flip;
  flip[1][1] *= -1;
  flip[2][2] *= -1;
  glm::mat4 in_gl_coords = flip * glm::mat4(in_pose);
  glm::vec3 axis;
  float angle;
  glm::axisAngle(in_gl_coords, axis, angle);
  glm::rotate(out_pose, angle, axis);

  out_pose = glm::inverse(out_pose); //bouguet poses (from calibration) are grid poses so invert to get camera poses

}

PoseGrabber::PoseGrabber(const ConfigReader &reader, const std::string &output_dir) : BasePoseGrabber(output_dir) {

  self_name_ = "pose-grabber";
  checkSelfName(reader.get_element("name"));

  try{
    model_.reset(new Model());
    model_->LoadData(reader.get_element("model-file"));
  }
  catch (std::runtime_error){
    //e.g. no model
  }
  
  
  ifs_.open(reader.get_element("pose-file"));

  if (!ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("pose-file"));
  }

  ifs_.exceptions(std::ifstream::eofbit);

  save_dir_ = output_dir;

  ofs_file_ = save_dir_ + "/" + reader.get_element("output-pose-file");

  load_source_ = LoadSource::FILE;

}

bool PoseGrabber::LoadPose(const bool update_as_new){

  do_draw_ = false; //set to true only if we read a 'good' pose

  bool retval = false;
  if (load_source_ == LoadSource::FILE){

    if (update_as_new){
      retval = LoadFromFile();
    }

  }
 
  // update the model with the pose
  model_->SetBasePose(cached_model_pose_);

  return retval;

}

bool PoseGrabber::LoadFromFile(){
  //load the new pose (if requested).

  try{
    std::string line;
    int row = 0;
    while (1)
    {
      std::getline(ifs_, line);
      if (row == 4) break;
      if (line[0] == '#' || line.length() < 1) continue;
      std::stringstream ss(line);
      for (int col = 0; col < 4; ++col){
        float val;
        ss >> val;
        cached_model_pose_[col][row] = val;
      }
      row++;
    }

    //update the reference list of old tracks for drawing trajectories
    reference_frame_tracks_.push_back(cached_model_pose_);
    do_draw_ = true;

  }
  catch (std::ofstream::failure e){
    cached_model_pose_ = glm::mat4();
    do_draw_ = false;
    return false;
  }

  return true;

}

void PoseGrabber::WritePoseToStream()  {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, cannot open file"));

  ofs_ << model_->GetPose() << "\n";

  ofs_ << "\n";

}

void PoseGrabber::WritePoseToStream(const glm::mat4 &camera_pose)  {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, cannot open file"));

  ofs_ << glm::inverse(camera_pose) * (glm::mat4)model_->GetPose() << "\n";

  ofs_ << "\n";

}

BaseDaVinciPoseGrabber::BaseDaVinciPoseGrabber(const InstrumentType instrument_type, const ConfigReader &reader, const std::string &output_dir) : BasePoseGrabber(output_dir) {
  
  load_source_ = LoadSource::FILE;

  try{
    model_.reset(new davinci::DaVinciInstrument());
    if (instrument_type == LND)
      model_->LoadData(davinci::DaVinciInstrument::GetLargeNeedleDriver());
    else if (instrument_type == NO_INST)
      return;
    else
      throw std::runtime_error("Error no other instruments supported yet!");
  }
  catch (std::runtime_error){
    //no model (e.g. tracking camera)
  }

}

BaseDaVinciPoseGrabber::BaseDaVinciPoseGrabber(const InstrumentType instrument_type, davinci::DaVinciJoint target_joint, const std::string &output_dir, const LoadSource source) : BasePoseGrabber(output_dir), target_joint_(target_joint), source_(source) {
  
  if (target_joint_ == davinci::DaVinciJoint::PSM1 || target_joint_ == davinci::DaVinciJoint::PSM2 || target_joint_ == davinci::DaVinciJoint::PSM3){
    model_.reset(new davinci::DaVinciInstrument());
    if (instrument_type == LND)
      model_->LoadData(davinci::DaVinciInstrument::GetLargeNeedleDriver());
    else
      throw std::runtime_error("Error no other instruments supported yet!");
  }

}


DHDaVinciPoseGrabber::DHDaVinciPoseGrabber(const davinci::DaVinciJoint &joint, const std::string &output_dir, LoadSource source) : DHDaVinciPoseGrabber(joint, NO_INST, output_dir, source) {}


DHDaVinciPoseGrabber::DHDaVinciPoseGrabber(const davinci::DaVinciJoint &joint, const InstrumentType instrument_type, const std::string &output_dir, LoadSource source) : BaseDaVinciPoseGrabber(instrument_type, joint, output_dir, source) {

  if (load_source_ == FILE){
    //
  }
  else if (load_source_ == ROS){

  }
  else if (load_source_ == ISI){
#ifdef USE_ISI_API

    const ISI_UINT password = strtol("1234567a", 0, 16);
    ISI_STATUS status = isi_connect_ex("10.0.0.5", 5002, password);
    if (status != ISI_SUCCESS)
      throw(std::runtime_error("Error, could not connect to daVinci API!\n"));
    
    isi_subscribe_all_stream_fields();
    isi_subscribe_all_events();

    ISI_UINT ISI_API_RATE = 50;
    status = isi_start_stream(ISI_API_RATE);
    if (status != ISI_SUCCESS){
      isi_stop_stream();
      isi_disconnect();
      throw(std::runtime_error("Failed to start stream\n"));
    }
#else
    throw std::runtime_error("ISI Api support not built!");
#endif
  }
  else{
    throw std::runtime_error("Unsupported load type");
  }


}

DHDaVinciPoseGrabber::DHDaVinciPoseGrabber(const InstrumentType instrument_type, const ConfigReader &reader, const std::string &output_dir) : BaseDaVinciPoseGrabber(instrument_type, reader, output_dir) {

  self_name_ = "dh-davinci-grabber";
  checkSelfName(reader.get_element("name"));

  if (reader.get_element("joint") == "PSM1")
    target_joint_ = davinci::DaVinciJoint::PSM1;
  else if (reader.get_element("joint") == "PSM2")
    target_joint_ = davinci::DaVinciJoint::PSM2;
  else if (reader.get_element("joint") == "ECM")
    target_joint_ = davinci::DaVinciJoint::ECM;
  else
    throw std::runtime_error("Error, bad joint");

  switch (target_joint_){

  case davinci::DaVinciJoint::PSM1:
    num_base_joints_ = chain_.mSUJ1OriginSUJ1Tip.size();
    num_arm_joints_ = chain_.mPSM1OriginPSM1Tip.size();
    break;
  case davinci::DaVinciJoint::PSM2:
    num_base_joints_ = chain_.mSUJ2OriginSUJ2Tip.size();
    num_arm_joints_ = chain_.mPSM2OriginPSM2Tip.size();
    break;
  case davinci::DaVinciJoint::ECM:
    num_base_joints_ = chain_.mSUJ3OriginSUJ3Tip.size();
    num_arm_joints_ = 4;//chain_.mECM1OriginECM1Tip.size(); 
    break;
  }

  arm_offsets_ = std::vector<double>(num_arm_joints_, 0.0);
  base_offsets_ = std::vector<double>(num_base_joints_, 0.0);
  arm_joints_ = std::vector<double>(num_arm_joints_, 0.0);
  base_joints_ = std::vector<double>(num_base_joints_, 0.0);

  try{
    SetupOffsets(reader.get_element("base-offset"), reader.get_element("arm-offset"));
  }
  catch (std::runtime_error &){
    
  }

  base_ifs_.open(reader.get_element("base-joint-file"));
  if (!base_ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("base-joint-file"));
  }

  base_ifs_.exceptions(std::ifstream::eofbit);

  arm_ifs_.open(reader.get_element("arm-joint-file"));
  if (!arm_ifs_.is_open()){
    throw std::runtime_error("Error, could not open file: " + reader.get_element("arm-joint-file"));
  }

  arm_ifs_.exceptions(std::ifstream::eofbit);

  base_ofs_file_ = output_dir + "/" + reader.get_element("output-base-joint-file");
  arm_ofs_file_ = output_dir + "/" + reader.get_element("output-arm-joint-file");
  try{
    se3_ofs_file_ = output_dir + "/" + reader.get_element("output-se3-file");
  }
  catch (...){
    se3_ofs_file_ = output_dir + "/" + reader.get_element("output-se3"); //stupid old code 
  }
  
}

DHDaVinciPoseGrabber::~DHDaVinciPoseGrabber() { 

  if (load_source_ == ISI){
#ifdef USE_ISI_API
    isi_disconnect();
#endif
  }

}


void DHDaVinciPoseGrabber::SetupOffsets(const std::string &base_offsets, const std::string &arm_offsets){

  std::stringstream ss;
  
  param_modifier_->addText("", "label=`Edit the set up joints`");

  ss << base_offsets;
  for (size_t i = 0; i < base_offsets_.size(); ++i){
    ss >> base_offsets_[i];
    std::stringstream ss;
    ss << "SU Joint " << i;
    param_modifier_->addParam(ss.str(), &(base_offsets_[i]), "min=-10 max=10 step= 0.0001 keyIncr=z keyDecr=Z");
  }

  param_modifier_->addSeparator();
  param_modifier_->addText("", "label=`Edit the arm joints`");

  ss.clear();
  ss << arm_offsets;
  for (size_t i = 0; i < arm_offsets_.size(); ++i){
    ss >> arm_offsets_[i];
    std::stringstream ss;
    ss << "Joint " << i;
    if (i < 3)
      param_modifier_->addParam(ss.str(), &(arm_offsets_[i]), "min=-10 max=10 step= 0.0001 keyIncr=z keyDecr=Z");
    else
      param_modifier_->addParam(ss.str(), &(arm_offsets_[i]), "min=-10 max=10 step= 0.001 keyIncr=z keyDecr=Z");
  }

  SetOffsetsToNull();

}

glm::mat4 DHDaVinciPoseGrabber::GetPose(){

  if (target_joint_ == davinci::ECM){

    srvlib::davinci::ECMData ecm;

    for (std::size_t i = 0; i < base_joints_.size(); ++i){
      ecm.sj_joint_angles[i] = base_joints_[i] + base_offsets_[i];
    }

    for (std::size_t i = 0; i < arm_joints_.size(); ++i){
      ecm.jnt_pos[i] = arm_joints_[i] + arm_offsets_[i];
    }

    glm::mat4 base_pose;
    buildKinematicChainECM1(chain_, ecm, base_pose);
    model_->SetBasePose(base_pose);

    return model_->GetPose();

  }

  else if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){

    srvlib::davinci::PSMData psm;

    for (std::size_t i = 0; i < base_joints_.size(); ++i){
      psm.sj_joint_angles[i] = base_joints_[i] + base_offsets_[i];
    }

    #ifdef USE_ROS
    lock_.lock();
    #endif
    for (std::size_t i = 0; i < arm_joints_.size(); ++i){
      psm.jnt_pos[i] = arm_joints_[i] + arm_offsets_[i];
    }
    #ifdef USE_ROS
    lock_.unlock();
    #endif

    glm::mat4 base_pose;
    glm::vec4 articulation;
    if (target_joint_ == davinci::PSM1)
      buildKinematicChainPSM1(chain_, psm, base_pose, articulation[0], articulation[1], articulation[2], articulation[3]);
    else if (target_joint_ == davinci::PSM2)
      buildKinematicChainPSM2(chain_, psm, base_pose, articulation[0], articulation[1], articulation[2], articulation[3]);

    model_->EditPose(glm::quat(base_pose), project(base_pose[3]), glm::vec3(articulation[0], articulation[1], articulation[2]));

    return model_->GetShaftPose();

  }
  else{

    throw std::runtime_error("Error, bad joint type");

  }
}

bool DHDaVinciPoseGrabber::LoadPose(const bool update_as_new){

  if (update_as_new){
    if(load_source_ == FILE){
      if (!ReadDHFromFiles(base_joints_, arm_joints_))
	return false;
    }else if(load_source_ == ISI){
#ifdef USE_ISI_API
      LoadFromISI();
#else
      throw std::runtime_error("Error, ISI API is not supported!\n");
#endif
    }
  }

  //don't care about the return.
  GetPose();

  // update the list of previous poses for plotting trajectories.
  if (update_as_new){
    if (target_joint_ == davinci::ECM)
      reference_frame_tracks_.push_back(model_->GetPose());
    else
      reference_frame_tracks_.push_back(model_->GetShaftPose());
  }

  return true;

}


#ifdef USE_ROS
bool DHDaVinciPoseGrabber::SetPose(const sensor_msgs::JointState::ConstPtr& msg){

  lock_.lock();

  for(size_t i = 0; i < num_arm_joints_; ++i){

    arm_joints_[i] = msg->position[i];

  }

  lock_.unlock();

  return true;

}
#endif

#ifdef USE_ISI_API
bool DHDaVinciPoseGrabber::LoadFromISI(){

  try{

    ISI_MANIP_INDEX mid;
    if (target_joint_ == davinci::DaVinciJoint::PSM1){
      mid == ISI_PSM1; 
    }
    else if (target_joint_ == davinci::DaVinciJoint::PSM2){
      mid == ISI_PSM2;
    }
    else if (target_joint_ == davinci::DaVinciJoint::PSM3){
      mid == ISI_PSM3;
    }
    else if(target_joint_ == davinci::DaVinciJoint::ECM){
      mid == ISI_ECM;
    }
    else{
      throw std::runtime_error("Error, bad joint type!");
    }

    ISI_STREAM_FIELD stream_data;

    isi_get_stream_field(mid, ISI_JOINT_VALUES, &stream_data);
    for (int i = 0; i != stream_data.count; i++){
      arm_joints_[i] = stream_data.data[i];
    }
    

    isi_get_stream_field(mid, ISI_SUJ_JOINT_VALUES, &stream_data);
    for (int i = 0; i != stream_data.count; i++){
      base_joints_[i] = stream_data.data[i];
    }

  }
  catch (std::ifstream::failure){
    do_draw_ = false;
    return false;
  }

  return true;

}
#endif

bool DHDaVinciPoseGrabber::ReadDHFromFiles(std::vector<double> &psm_base_joints, std::vector<double> &psm_arm_joints){

  assert(num_arm_joints_ == psm_arm_joints.size());
  assert(num_base_joints_ == psm_base_joints.size());

  try{
    for (int i = 0; i < num_arm_joints_; ++i){
      double x;
      arm_ifs_ >> x;
      psm_arm_joints[i] = x;
    }

    for (int i = 0; i < num_base_joints_; ++i){
      double x;
      base_ifs_ >> x;
      psm_base_joints[i] = x;

    }

  }
  catch (std::ifstream::failure){
    do_draw_ = false;
    return false;
  }

  return true;

}

void DHDaVinciPoseGrabber::WritePoseToStream()  {

  if (!se3_ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    se3_ofs_.open(se3_ofs_file_);
    arm_ofs_.open(arm_ofs_file_);
    base_ofs_.open(base_ofs_file_);
  }

  if (!se3_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!arm_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!base_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  se3_ofs_ << WriteSE3ToString(model_->GetShaftPose()) << "\n";
  for (size_t i = 4; i < arm_joints_.size(); ++i){
    se3_ofs_ << arm_joints_[i] + arm_offsets_[i] << "\n";
  }
  se3_ofs_ << std::endl;

  for (size_t i = 0; i < arm_joints_.size(); ++i){
    arm_ofs_ << arm_joints_[i] + arm_offsets_[i] << " ";
  }
  arm_ofs_ << std::endl;

  for (size_t i = 0; i < base_joints_.size(); ++i){
    base_ofs_ << base_joints_[i] + base_offsets_[i] << " ";
  }
  base_ofs_ << std::endl;

}

void DHDaVinciPoseGrabber::SetOffsetsToNull() {

  for (size_t i = 0; i < base_offsets_.size(); ++i){

    base_offsets_[i] = 0;

  }

  for (size_t i = 0; i < arm_offsets_.size(); ++i){

    arm_offsets_[i] = 0;

  }

}

void DHDaVinciPoseGrabber::WritePoseToStream(const glm::mat4 &camera_pose)  {

  if (!se3_ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    se3_ofs_.open(se3_ofs_file_);
    arm_ofs_.open(arm_ofs_file_);
    base_ofs_.open(base_ofs_file_);
  }

  if (!se3_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!arm_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));
  if (!base_ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  se3_ofs_ << WriteSE3ToString( glm::inverse(camera_pose) * (glm::mat4)model_->GetShaftPose()) << "\n";
  for (size_t i = 4; i < arm_joints_.size(); ++i){
    se3_ofs_ << arm_joints_[i] + arm_offsets_[i] << "\n";
  }
  se3_ofs_ << std::endl;

  for (size_t i = 0; i < arm_joints_.size(); ++i){
    arm_ofs_ << arm_joints_[i] + arm_offsets_[i] << " ";
  }
  arm_ofs_ << std::endl;

  for (size_t i = 0; i < base_joints_.size(); ++i){
    base_ofs_ << base_joints_[i] + base_offsets_[i] << " ";
  }
  base_ofs_ << std::endl;

}

void SE3DaVinciPoseGrabber::SetupOffsets(const std::string &base_offsets, const std::string &arm_offsets){

  //if (base_offsets.size() != 6 || arm_offsets.size() != 3) {
  //  ci::app::console() << "Error, base offsets size should be 6 and arm offsets size should be 3\n" << std::endl;
  //  throw std::runtime_error("");
  //}

  std::stringstream ss;
  std::stringstream ss2;

  param_modifier_->addText("", "label=`Edit the 6 DOF pose joints`");

  ss << base_offsets;
  ss >> x_rotation_offset_;
  param_modifier_->addParam("X rotation offset", &x_rotation_offset_, "min=-10 max=10 step= 0.01 keyIncr=r keyDecr=R");
  

  ss << base_offsets;
  ss >> y_rotation_offset_;
  param_modifier_->addParam("Y rotation offset", &y_rotation_offset_, "min=-10 max=10 step= 0.01 keyIncr=p keyDecr=P");

  ss << base_offsets;
  ss >> z_rotation_offset_;
  param_modifier_->addParam("Z rotation offset", &z_rotation_offset_, "min=-10 max=10 step= 0.01 keyIncr=y keyDecr=Y");

  ss << base_offsets;
  ss >> x_translation_offset_;
  param_modifier_->addParam("X translation offset", &x_translation_offset_, "min=-10 max=10 step= 0.001 keyIncr=x keyDecr=X");

  ss << base_offsets;
  ss >> y_translation_offset_;
  param_modifier_->addParam("Y translation offset", &y_translation_offset_, "min=-10 max=10 step= 0.001 keyIncr=y keyDecr=Y");

  ss << base_offsets;
  ss >> z_translation_offset_;
  param_modifier_->addParam("Z translation offset", &z_translation_offset_, "min=-10 max=10 step= 0.001 keyIncr=z keyDecr=Z");

  param_modifier_->addSeparator();
  param_modifier_->addText("", "label=`Edit the wrist joints`");

  ss.clear();
  ss << arm_offsets;
  for (size_t i = 0; i < wrist_offsets_.size(); ++i){
    ss >> wrist_offsets_[i];
    std::stringstream ss2;
    ss2 << "Joint " << i;
    if (i < 3)
      param_modifier_->addParam(ss2.str(), &(wrist_offsets_[i]), "min=-10 max=10 step= 0.0001 keyIncr=z keyDecr=Z");
    else
      param_modifier_->addParam(ss2.str(), &(wrist_offsets_[i]), "min=-10 max=10 step= 0.001 keyIncr=z keyDecr=Z");
  }

  SetOffsetsToNull();

}

void SE3DaVinciPoseGrabber::SetOffsetsToNull() {

  entire_x_rotation_offset_ = 0.0f; 
  entire_y_rotation_offset_ = 0.0f;
  entire_z_rotation_offset_ = 0.0f;

  x_rotation_offset_ = 0.0f;
  y_rotation_offset_ = 0.0f;
  z_rotation_offset_ = 0.0f;
  x_translation_offset_ = 0.0f;
  y_translation_offset_ = 0.0f;
  z_translation_offset_ = 0.0f;

  for (int i = 0; i < num_wrist_joints_; ++i){
    wrist_offsets_[i] = 0.0f;
  }

}

SE3DaVinciPoseGrabber::SE3DaVinciPoseGrabber(const InstrumentType instrument_type, const ConfigReader &reader, const std::string &output_dir, bool check_type) : BaseDaVinciPoseGrabber(instrument_type, reader, output_dir) {

  if (check_type){
    self_name_ = "se3-davinci-grabber";
    checkSelfName(reader.get_element("name"));
  }

  std::string rotation_type = reader.get_element("rotation-type");
  if (rotation_type == "euler") rotation_type_ = LoadType::EULER;
  else if (rotation_type == "quaternion") rotation_type_ = LoadType::QUATERNION;
  else if (rotation_type == "matrix") rotation_type_ = LoadType::MATRIX;
  else throw std::runtime_error("");

  if (reader.get_element("joint") == "PSM1")
    target_joint_ = davinci::DaVinciJoint::PSM1;
  else if (reader.get_element("joint") == "PSM2")
    target_joint_ = davinci::DaVinciJoint::PSM2;
  else if (reader.get_element("joint") == "ECM")
    target_joint_ = davinci::DaVinciJoint::ECM;
  else
    throw std::runtime_error("Error, bad joint");

  ifs_.open(reader.get_element("pose-file"));
  if (!ifs_.is_open()) throw std::runtime_error("Could not open file!\n");
  ofs_file_ = output_dir + "/" + reader.get_element("output-pose-file");

  num_wrist_joints_ = 3; //should this load from config file?

  wrist_dh_params_ = std::vector<double>(num_wrist_joints_, 0.0);
  wrist_offsets_ = std::vector<float>(num_wrist_joints_, 0.0);

  try{
    SetupOffsets(reader.get_element("base-offset"), reader.get_element("arm-offset"));
  }
  catch (std::runtime_error &){

  }

  current_user_supplied_offset_ = glm::mat4();

}

glm::mat4 SE3DaVinciPoseGrabber::MatrixFromIntrinsicEulers(float xRotation, float yRotation, float zRotation, const std::string &order) const {

  float cosx = ci::math<float>::cos(xRotation);
  float cosy = ci::math<float>::cos(yRotation);
  float cosz = ci::math<float>::cos(zRotation);
  float sinx = ci::math<float>::sin(xRotation);
  float siny = ci::math<float>::sin(yRotation);
  float sinz = ci::math<float>::sin(zRotation);

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
  //xyz
  //ci::Matrix33f r = zRotationMatrix * yRotationMatrix * xRotationMatrix;

  //zyx
  if (order == "zyx")
    r = xRotationMatrix * yRotationMatrix * zRotationMatrix;
  else if (order == "xyz")
    r = zRotationMatrix * yRotationMatrix * xRotationMatrix;
  else if (order == "xzy")
    r = yRotationMatrix * zRotationMatrix * xRotationMatrix;
  else
    throw std::runtime_error("");

  glm::mat4 rr(r);
  rr[3][3] = 1.0f;
  return rr;

}

/*
inline glm::quat QuaternionFromEulers(float xRotation, float yRotation, float zRotation){

  zRotation *= 0.5f;
  yRotation *= 0.5f;
  xRotation *= 0.5f;

  // get sines and cosines of half angles
  float Cx = ci::math<float>::cos(xRotation);
  float Sx = ci::math<float>::sin(xRotation);

  float Cy = ci::math<float>::cos(yRotation);
  float Sy = ci::math<float>::sin(yRotation);
                            
  float Cz = ci::math<float>::cos(zRotation);
  float Sz = ci::math<float>::sin(zRotation);

  glm::quat r;
  // multiply it out
  r.w = Cx*Cy*Cz + Sx*Sy*Sz;
  r.v.x = Sx*Cy*Cz - Cx*Sy*Sz;
  r.v.y = Cx*Sy*Cz + Sx*Cy*Sz;
  r.v.z = Cx*Cy*Sz - Sx*Sy*Cz;

  return r.normalized();
}
*/

glm::vec3 SE3DaVinciPoseGrabber::GetXYZEulersFromQuaternion(const glm::quat &quaternion) const {
    
  glm::vec3 angles;
//
//  /*
//  angles(1,iel) = atan2(2.*(q(iel).e(2).*q(iel).e(1)+ ...
//                     q(iel).e(4).*q(iel).e(3)),(q(iel).e(1).^2- ...
//                        q(iel).e(2).^2-q(iel).e(3).^2+q(iel).e(4).^2));
//                    angles(2,iel) = asin(2.*(q(iel).e(3).*q(iel).e(1)- ...
//                        q(iel).e(2).*q(iel).e(4)));
//                    angles(3,iel) = atan2(2.*(q(iel).e(2).*q(iel).e(3)+ ...
//                        q(iel).e(4).*q(iel).e(1)),(q(iel).e(1).^2+ ...
//                        q(iel).e(2).^2-q(iel).e(3).^2-q(iel).e(4).^2));  
//  */

  angles[0] = atan2(2 * (quaternion.x*quaternion.w + quaternion.z*quaternion.y), (quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z));
  angles[1] = asin(2 * (quaternion.y*quaternion.w - quaternion.x*quaternion.z));
  angles[2] = atan2(2 * (quaternion.x*quaternion.y + quaternion.z*quaternion.w), (quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z));  
  return angles;

}

glm::vec3 SE3DaVinciPoseGrabber::GetXZYEulersFromQuaternion(const glm::quat &quaternion) const {

  /*angles(1, iel) = atan2(2.*(q(iel).e(2).*q(iel).e(4) + ...
    q(iel).e(3).*q(iel).e(1)), (q(iel).e(1). ^ 2 + ...
    q(iel).e(2). ^ 2 - q(iel).e(3). ^ 2 - q(iel).e(4). ^ 2));
    angles(2, iel) = asin(2.*(q(iel).e(4).*q(iel).e(1) - ...
    q(iel).e(2).*q(iel).e(3)));
    angles(3, iel) = atan2(2.*(q(iel).e(2).*q(iel).e(1) + ...
    q(iel).e(3).*q(iel).e(4)), (q(iel).e(1). ^ 2 - ...
    q(iel).e(2). ^ 2 + q(iel).e(3). ^ 2 - q(iel).e(4). ^ 2));
    */
  glm::vec3 angles;
  angles[0] = atan2(2 * (quaternion.x*quaternion.z - quaternion.x*quaternion.y), (quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z));
  angles[1] = asin(2 * (quaternion.w*quaternion.z - quaternion.y*quaternion.x));
  angles[2] = atan2(2 * (quaternion.x*quaternion.w + quaternion.y*quaternion.z), (quaternion.w * quaternion.w - quaternion.x * quaternion.x + quaternion.y * quaternion.y - quaternion.z * quaternion.z));

  return angles;
}

glm::vec3 SE3DaVinciPoseGrabber::GetZYXEulersFromQuaternion(const glm::quat &quaternion) const {

  glm::vec3 angles;

  //angles(1,iel) = atan2(2.*(q(iel).e(4).*q(iel).e(1)- ...
  //q(iel).e(2).*q(iel).e(3)), (q(iel).e(1). ^ 2 + ...
  //  q(iel).e(2). ^ 2 - q(iel).e(3). ^ 2 - q(iel).e(4). ^ 2));
  //  angles(2, iel) = asin(2.*(q(iel).e(2).*q(iel).e(4) + ...
  //    q(iel).e(3).*q(iel).e(1)));
  //  angles(3, iel) = atan2(2.*(q(iel).e(2).*q(iel).e(1) - ...
  //    q(iel).e(3).*q(iel).e(4)), (q(iel).e(1). ^ 2 - ...
  //   q(iel).e(2). ^ 2 - q(iel).e(3). ^ 2 + q(iel).e(4). ^ 2));

  angles[0] = atan2(2 * (quaternion.z*quaternion.w - quaternion.x*quaternion.y), (quaternion.w * quaternion.w + quaternion.x * quaternion.x - quaternion.y * quaternion.y - quaternion.z * quaternion.z));
  angles[1] = asin(2 * (quaternion.x*quaternion.z + quaternion.y*quaternion.w));
  angles[2] = atan2(2 * (quaternion.x*quaternion.w - quaternion.y*quaternion.z), 
    (quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z));

  return angles;

}

void SE3DaVinciPoseGrabber::SetPose(glm::vec3 &translation, glm::quat &rotation, glm::vec3 &articulation){

  translation_ = translation;
  rotation_ = rotation;
  for (int i = 0; i < 3; ++i){
    wrist_dh_params_[i] = articulation[i];
  }

  shaft_pose_ = glm::mat4(rotation_) * current_user_supplied_offset_;

  srvlib::math::set_translate(shaft_pose_, translation_);

  model_->SetBasePose(shaft_pose_);

  model_->EditPose(glm::quat(shaft_pose_), project(shaft_pose_[3]), glm::vec3(wrist_dh_params_[0], wrist_dh_params_[1], wrist_dh_params_[2]));

}



void SE3DaVinciPoseGrabber::GetPose(glm::vec3 &translation, glm::vec3 &rotation, glm::vec3 &articulation) const {
  
  translation = translation_;

  rotation = GetZYXEulersFromQuaternion(rotation_);

  for (int i = 0; i < 3; ++i)
    articulation[i] = wrist_dh_params_[i];

}

void SE3DaVinciPoseGrabber::EditPose(glm::vec3 &translation, glm::vec3 &rotation, glm::vec3 &articulation){

  //translation_[0] += 4;
  
  translation_ = translation;

  glm::vec3 current_rotation = GetZYXEulersFromQuaternion(rotation_ * glm::quat(current_user_supplied_offset_));
 
  for (int i = 0; i < 3; ++i)
    wrist_dh_params_[i] = articulation[i];

  shaft_pose_ = MatrixFromIntrinsicEulers(current_rotation[2], current_rotation[1], current_rotation[0], "zyx") * current_user_supplied_offset_;

  srvlib::math::set_translate(shaft_pose_, translation_);
  model_->SetBasePose(shaft_pose_);

  model_->EditPose(glm::quat(shaft_pose_), project(shaft_pose_[3]), glm::vec3(wrist_dh_params_[0], wrist_dh_params_[1], wrist_dh_params_[2]));
  
}

bool SE3DaVinciPoseGrabber::LoadPoseAsQuaternion(){

  try{
    std::string line;
    int row = 0;

    glm::vec3 articulation;
    //remember - also set psmatend rotation angle for tip to +- val rather than +- 0.5*val. aslo skipping frist 59 frames.


    for (int i = 0; i < 3; ++i){
      ifs_ >> translation_[i];
    }

    for (int i = 0; i < 4; ++i){
      ifs_ >> rotation_[i];
    }

    for (int i = 0; i < 3; ++i){
      ifs_ >> articulation[i];
    }
    for (int i = 0; i < 3; ++i){
      wrist_dh_params_[i] = articulation[i];
    }

    //test for visualization

    //glm::vec3 eulers = GetZYXEulersFromQuaternion(rotation_);
    //static float increment = 0.05;
    //increment = increment + 0.05;

    //eulers[2] += increment;

    //glm::mat4 qqnew = MatrixFromIntrinsicEulers(3.141592 / 2 + 0 * eulers[0], 0 * eulers[1], eulers[2]);
    //translation_[0] *= 0;
    //translation_[1] *= 0;
    
    //rotation_ = qqnew;

    glm::mat4 rotation_m(rotation_);
    shaft_pose_ = rotation_m * current_user_supplied_offset_;

    return true;
  }
  catch (std::ifstream::failure e){
    shaft_pose_ = glm::mat4();
    do_draw_ = false;
    return false;
  }
}

bool SE3DaVinciPoseGrabber::LoadPoseAsMatrix(){

  throw std::runtime_error("");

  try{
    std::string line;
    int row = 0;

    glm::vec3 articulation;
    //remember - also set psmatend rotation angle for tip to +- val rather than +- 0.5*val. aslo skipping frist 59 frames.

    glm::mat3 rotation_matrix;
    glm::vec3 translation;

    for (int i = 0; i < 3; ++i){
      std::string line;
      std::getline(ifs_, line);


    }

 
    translation_ = translation;
    rotation_ = glm::quat(rotation_matrix);

    glm::mat4 rotation_m(rotation_);
    shaft_pose_ = rotation_m *current_user_supplied_offset_;

    return true;
  }
  catch (std::ifstream::failure e){
    shaft_pose_ = glm::mat4();
    do_draw_ = false;
    return false;
  }
}

bool SE3DaVinciPoseGrabber::LoadPoseAsEulerAngles(){

  try{
    std::string line;
    int row = 0;

    glm::vec3 eulers;
    glm::vec3 articulation;
    //remember - also set psmatend rotation angle for tip to +- val rather than +- 0.5*val. aslo skipping frist 59 frames.

    for (int i = 0; i < 3; ++i){
      ifs_ >> translation_[i];
     }

    for (int i = 0; i < 3; ++i){
      ifs_ >> eulers[i];
    }

    for (int i = 0; i < 3; ++i){
      ifs_ >> articulation[i];
    }
    for (int i = 0; i < 3; ++i){
      wrist_dh_params_[i] = articulation[i];
    }
    
    glm::mat4 rotation_matrix = MatrixFromIntrinsicEulers(eulers[0], eulers[1], eulers[2], "zyx");
    
    rotation_ = glm::quat(rotation_matrix);

    glm::mat4 rotation_m(rotation_);
    shaft_pose_ = rotation_m *current_user_supplied_offset_;

    return true;
  }
  catch (std::ifstream::failure e){
    shaft_pose_ = glm::mat4();
    do_draw_ = false;
    return false;
  }
}



bool SE3DaVinciPoseGrabber::LoadPose(const bool update_as_new){
  
  do_draw_ = false; //set to true only if we read a 'good' pose

  assert(num_wrist_joints_ == wrist_dh_params_.size());

  if (update_as_new){

    if (ifs_.eof()) return false;

    bool load_success = false;

    if (rotation_type_ == LoadType::QUATERNION)
      load_success = LoadPoseAsQuaternion();

    else if (rotation_type_ == LoadType::EULER)
      load_success = LoadPoseAsEulerAngles();

    else if (rotation_type_ == LoadType::MATRIX)
      load_success = LoadPoseAsMatrix();

    else
      return false;

    if (!load_success) return false;

  }

  
  auto offset = MatrixFromIntrinsicEulers(x_rotation_offset_ - entire_x_rotation_offset_, y_rotation_offset_ - entire_y_rotation_offset_, z_rotation_offset_ - entire_z_rotation_offset_, "zyx");

  entire_x_rotation_offset_ = x_rotation_offset_;
  entire_y_rotation_offset_ = y_rotation_offset_;
  entire_z_rotation_offset_ = z_rotation_offset_;

  current_user_supplied_offset_ = current_user_supplied_offset_ * offset;
  shaft_pose_ = shaft_pose_ * offset;
  
  srvlib::math::set_translate(shaft_pose_, (translation_ + glm::vec3(x_translation_offset_, y_translation_offset_, z_translation_offset_)));

  do_draw_ = true;

  // update the list of previous poses for plotting trajectories.
  reference_frame_tracks_.push_back(shaft_pose_);

  model_->EditPose(glm::quat(shaft_pose_), project(shaft_pose_[3]), glm::vec3(wrist_dh_params_[0], wrist_dh_params_[1], wrist_dh_params_[2]));

  return true;

}

void SE3DaVinciPoseGrabber::WritePoseToStream() {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  ofs_ << WriteSE3ToString(model_->GetShaftPose()) << "\n";
  
  for (size_t i = 0; i < wrist_dh_params_.size(); ++i){
    ofs_ << wrist_dh_params_[i] << "\n";
  }
  ofs_ << "\n";
}

void SE3DaVinciPoseGrabber::WritePoseToStream(const glm::mat4 &camera_pose)  {

  if (!ofs_.is_open()) {
    if (!boost::filesystem::exists(save_dir_)) {
      boost::filesystem::create_directory(save_dir_);
    }
    ofs_.open(ofs_file_);
  }

  if (!ofs_.is_open()) throw(std::runtime_error("Error, could not open file"));

  ofs_ << WriteSE3ToString(glm::inverse(camera_pose) * (glm::mat4)model_->GetShaftPose()) << "\n";
  for (size_t i = 0; i < wrist_dh_params_.size(); ++i){
    ofs_ << wrist_dh_params_[i] << "\n";
  }
  ofs_ << "\n";
  
}

void DHDaVinciPoseGrabber::GetModelPose(glm::mat4 &head, glm::mat4 &clasper_left, glm::mat4 &clasper_right){
	
	if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){

		srvlib::davinci::PSMData psm;

    for (std::size_t i = 0; i < base_joints_.size(); ++i){
			psm.sj_joint_angles[i] = base_joints_[i] + base_offsets_[i];
		}

		for (std::size_t i = 0; i < arm_joints_.size(); ++i){
			psm.jnt_pos[i] = arm_joints_[i] + arm_offsets_[i];
		}

    glm::mat4 base_pose;
    glm::vec4 articulation;
		if (target_joint_ == davinci::PSM1)
      buildKinematicChainPSM1(chain_, psm, base_pose, articulation[0], articulation[1], articulation[2], articulation[3]);
		else if (target_joint_ == davinci::PSM2)          
      buildKinematicChainPSM2(chain_, psm, base_pose, articulation[0], articulation[1], articulation[2], articulation[3]);

    model_->EditPose(glm::quat(base_pose), project(base_pose[3]), glm::vec3(articulation[0], articulation[1], articulation[2]));

		head = model_->GetHeadPose();
    clasper_left = model_->GetClasper1Pose();
    clasper_right = model_->GetClasper2Pose();

	}
	else{

		throw std::runtime_error("Error, bad joint type");

	}

}

void SE3DaVinciPoseGrabber::GetModelPose(glm::mat4 &head, glm::mat4 &clasper_left, glm::mat4 &clasper_right){

  if (target_joint_ == davinci::PSM1 || target_joint_ == davinci::PSM2){
  
    //model_->Shaft().transform_ = shaft_pose_;

    srvlib::davinci::PSMData psm;
    for (size_t i = 0; i < num_wrist_joints_; ++i){
      psm.jnt_pos[i] = wrist_dh_params_[i];
    }
    
    //if (target_joint_ == davinci::PSM1)
    //  buildKinematicChainPSM1(chain_, psm, model_->Shaft().transform_, model_->Head().transform_, model_->Clasper1().transform_, model_->Clasper2().transform_);
    //else if (target_joint_ == davinci::PSM2)
    //  buildKinematicChainPSM2(chain_, psm, model_->Shaft().transform_, model_->Head().transform_, model_->Clasper1().transform_, model_->Clasper2().transform_);

    //head = model_->Head().transform_;
    //clasper_left = model_->Clasper1().transform_;
    //clasper_right = model_->Clasper2().transform_;

  }
  else{

    throw std::runtime_error("Error, bad joint type");

  }

}

void SE3DaVinciPoseGrabber::DrawBody(){

  model_->DrawBody();

}

void SE3DaVinciPoseGrabber::DrawHead(){

  model_->DrawHead();
  model_->DrawLeftClasper();
  model_->DrawRightClasper();

}

void DHDaVinciPoseGrabber::DrawBody(){

	model_->DrawBody();

}

void DHDaVinciPoseGrabber::DrawHead(){

	model_->DrawHead();
  model_->DrawLeftClasper();
  model_->DrawRightClasper();

}

bool QuaternionDaVinciPoseGrabber::LoadPose(const bool update_as_new){

  do_draw_ = false; //set to true only if we read a 'good' pose

  //load the new pose (if requested).
  if (update_as_new){
    try{
      std::string line;
      int row = 0;
      while (ifs_.good()){
        std::getline(ifs_, line);
        if (line[0] == '#' || line.length() < 1) continue;
        break;
      }
      std::stringstream ss(line);
      
      glm::vec3 translation;
      for (size_t col = 0; col < 3; ++col){
        float val;
        ss >> val;
        translation[col] = val;
      }

      

      glm::vec4 quats;
      for (size_t col = 0; col < 4; ++col){
        float val;
        ss >> val;
        quats[col] = val;
      }

      shaft_pose_ = glm::mat4(glm::quat(quats[0], quats[1], quats[2], quats[3]));
      srvlib::math::set_translate(shaft_pose_, translation);
      

      //update the reference list of old tracks for drawing trajectories
      reference_frame_tracks_.push_back(shaft_pose_);
      do_draw_ = true;

    }
    catch (std::ofstream::failure e){
      shaft_pose_ = glm::mat4();
      do_draw_ = false;
      return false;
    }
  }

  model_->EditPose(glm::quat(shaft_pose_), project(shaft_pose_[3]), glm::vec3(wrist_dh_params_[0], wrist_dh_params_[1], wrist_dh_params_[2]));

  return true;


}

QuaternionDaVinciPoseGrabber::QuaternionDaVinciPoseGrabber(const InstrumentType instrument_type, const ConfigReader &reader, const std::string &output_dir) : SE3DaVinciPoseGrabber(instrument_type, reader, output_dir, false) {

  self_name_ = "quaternion-pose-grabber";
  checkSelfName(reader.get_element("name"));

}
