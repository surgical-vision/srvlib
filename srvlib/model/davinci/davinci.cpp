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

This file was based on work by Philip Pratt, Imperial College London. Used with permission.

**/

#include <srvlib/model/davinci/davinci.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

using namespace srvlib::davinci;

const float PI = 3.14159265358979323846f;
const float PI_2 = 1.57079632679489661923f;
const float PI_4 = 0.785398163397448309616f;

// da Vinci coordinates are in meters but we always work in mm.
const double SCALE = 1000;

void DaVinciInstrument::UpdatePose(const glm::vec4 &quaternion_update, const glm::vec3 &translation_update, const glm::vec3 &articulation_update){



}

void DaVinciInstrument::EditPose(const glm::quat &rotation, const glm::vec3 &translation, const glm::vec3 &articulation){



}

void DaVinciInstrument::UpdatePose(const glm::vec3 &euler_update, const std::string &euler_order, const bool use_intrinsic_eulers, const glm::vec3 &translation_update, const glm::vec3 &articulation_update) {


}

void DaVinciInstrument::EditPose(const glm::vec3 &euler_rotation, const std::string &euler_order, const bool use_intrinsic_eulers, const glm::vec3 &translation, const glm::vec3 &articulation){



}


void DaVinciInstrument::Draw() const {

  InternalDraw(shaft_);
  InternalDraw(head_);
  InternalDraw(clasper1_);
  InternalDraw(clasper2_);

}

void DaVinciInstrument::LoadData(const std::string &datafile_path){

  ci::JsonTree tree = OpenFile(datafile_path);

  LoadComponent(tree.getChild("shaft"), shaft_, boost::filesystem::path(datafile_path).parent_path().string());
  LoadComponent(tree.getChild("head"), head_, boost::filesystem::path(datafile_path).parent_path().string());
  LoadComponent(tree.getChild("clasper1"), clasper1_, boost::filesystem::path(datafile_path).parent_path().string());
  LoadComponent(tree.getChild("clasper2"), clasper2_, boost::filesystem::path(datafile_path).parent_path().string());

}

std::vector<glm::mat4> DaVinciInstrument::GetTransformSet() const{
  return std::vector<glm::mat4>({ shaft_.transform_, head_.transform_, clasper1_.transform_, clasper2_.transform_ });
}

void DaVinciInstrument::SetTransformSet(const std::vector<glm::mat4> &transforms){

  assert(transforms.size() == 4);
  shaft_.transform_ = transforms[0];
  head_.transform_ = transforms[1];
  clasper1_.transform_ = transforms[2];
  clasper2_.transform_ = transforms[3];

}

void DaVinciInstrument::DrawBody() const{

  InternalDraw(shaft_);

}

void DaVinciInstrument::DrawLeftClasper() const{

  InternalDraw(clasper1_);

}


void DaVinciInstrument::DrawRightClasper() const{

  InternalDraw(clasper2_);

}

void DaVinciInstrument::DrawHead() const{

  InternalDraw(head_);

}


DaVinciKinematicChain::DaVinciKinematicChain(void){

  // General transformation from world origin to SUJ1 origin (assumes alpha cart)
  mWorldOriginSUJ1Origin.push_back(GeneralFrame(-0.1016f, -0.1016f, 0.43f, -1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from SUJ1 origin to SUJ1 tip (assumes alpha cart)
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::PRISMATIC, 0.08979f, 0.0f, 0.0f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, 0.0f, 0.4166f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::ROTARY, 0.4318f, 0.0f, 0.14288f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.4318f, 0.0f, -0.1302f, PI_2));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.4089f, 0.0f));
  mSUJ1OriginSUJ1Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, 0.0f, -PI_2, -0.1029f, -PI_2));

  // General transformationfrom SUJ1 tip to PSM1 origin
  mSUJ1TipPSM1Origin.push_back(GeneralFrame(0.478f, 0.0f, 0.1524f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from PSM1 origin to PSM1 tip (assumes needle driver instrument)
  const float PSM1_lenRCC = 0.4318f;
  const float PSM1_ToolLen = 0.4159f;
  const float PSM1_PitchToYaw = 0.009f;
  const float PSM1_YawToCtrlPnt = 0.0f;
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.0f, PI_2));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::PRISMATIC, 0.0f, PI_2, -PSM1_lenRCC, 0.0f));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.0f, 0.0f, PSM1_ToolLen, 0.0f));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, PSM1_PitchToYaw, -PI_2, 0.0f, -PI_2));
  mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::FIXED, 0.0f, -PI_2, PSM1_YawToCtrlPnt, 0.0f));
  //mPSM1OriginPSM1Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::ROTARY, 0.0f, -PI_2, PSM1_YawToCtrlPnt, 0.0f));

  // General transformation from world origin to SUJ2 origin (assumes alpha cart)
  mWorldOriginSUJ2Origin.push_back(GeneralFrame(0.1016f, -0.1016f, 0.43f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from SUJ2 origin to SUJ2 tip (assumes alpha cart)
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::PRISMATIC, 0.08979f, 0.0f, 0.0f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, 0.0f, 0.4166f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::ROTARY, 0.4318f, 0.0f, 0.14288f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.4318f, 0.0f, -0.1302f, PI_2));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.4089f, 0.0f));
  mSUJ2OriginSUJ2Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, 0.0f, -PI_2, -0.1029f, -PI_2));

  // General transformationfrom SUJ2 tip to PSM2 origin
  mSUJ2TipPSM2Origin.push_back(GeneralFrame(0.478f, 0.0f, 0.1524f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from PSM2 origin to PSM2 tip (assumes needle driver instrument)
  const float PSM2_lenRCC = 0.4318f;
  const float PSM2_ToolLen = 0.4159f;
  const float PSM2_PitchToYaw = 0.009f;
  const float PSM2_YawToCtrlPnt = 0.0f;
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.0f, PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::PRISMATIC, 0.0f, PI_2, -PSM2_lenRCC, 0.0f));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.0f, 0.0f, PSM2_ToolLen, 0.0f));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::ROTARY, PSM2_PitchToYaw, -PI_2, 0.0f, -PI_2));
  mPSM2OriginPSM2Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::FIXED, 0.0f, -PI_2, PSM2_YawToCtrlPnt, 0.0f));

  // Denavit-Hartenberg parameters from world origin to SUJ3 origin (assumes alpha cart)
  mWorldOriginSUJ3Origin.push_back(GeneralFrame(0.0f, 0.0f, 0.43f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from SUJ3 origin to SUJ3 tip (assumes alpha cart)
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::PRISMATIC, 0.08979f, 0.0f, 0.0f, 0.0f));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, 0.0f, 0.4166f, 0.0f));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::ROTARY, 0.4318f, 0.0f, 0.14288f, 0.0f));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.4318f, 0.0f, -0.34588f, PI_2));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::FIXED, 0.0f, -PI_4, 0.0f, PI_2));
  mSUJ3OriginSUJ3Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::FIXED, -0.06641f, 0.0f, 0.0f, 0.0f));

  // Denavit-Hartenberg parameters from SUJ3 tip to ECM1 origin
  mSUJ3TipECM1Origin.push_back(GeneralFrame(0.6126f, 0.0f, 0.1016f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f));

  // Denavit-Hartenberg parameters from ECM1 origin to ECM1 tip (assumes Olympus 0 degree stereo scope)
  const float ECM1_lenRCC = 0.3822f;
  const float ECM1_ScopeLen = 0.3828f;
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(1, JointTypeEnum::ROTARY, 0.0f, PI_2, 0.0f, PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(2, JointTypeEnum::ROTARY, 0.0f, -PI_2, 0.0f, -PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(3, JointTypeEnum::PRISMATIC, 0.0f, PI_2, -ECM1_lenRCC, 0.0f));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(4, JointTypeEnum::ROTARY, 0.0f, 0.0f, ECM1_ScopeLen, 0.0f));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(5, JointTypeEnum::FIXED, 0.0f, -PI_2, 0.0f, -PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(6, JointTypeEnum::FIXED, 0.0f, -PI_2, 0.0f, -PI_2));
  mECM1OriginECM1Tip.push_back(DenavitHartenbergFrame(7, JointTypeEnum::FIXED, 0.0f, -PI_2, 0.0f, 0.0f));

}

void srvlib::davinci::extendChain(const GeneralFrame& frame, GLfloat* A) {

  GLfloat G[16];

  G[_00] = frame.mR00;
  G[_10] = frame.mR10;
  G[_20] = frame.mR20;
  G[_30] = 0.0;
  G[_01] = frame.mR01;
  G[_11] = frame.mR11;
  G[_21] = frame.mR21;
  G[_31] = 0.0;
  G[_02] = frame.mR02;
  G[_12] = frame.mR12;
  G[_22] = frame.mR22;
  G[_32] = 0.0;
  G[_03] = frame.mX * SCALE;
  G[_13] = frame.mY * SCALE;
  G[_23] = frame.mZ * SCALE;
  G[_33] = 1.0;

  glhMultMatrixRight(G, A);

}

void srvlib::davinci::glhDenavitHartenberg(GLfloat a, GLfloat alpha, GLfloat d, GLfloat theta, GLfloat* A) {

  GLfloat sa = sin(alpha);
  GLfloat ca = cos(alpha);
  GLfloat st = sin(theta);
  GLfloat ct = cos(theta);

  A[_00] = ct;
  A[_10] = ca * st;
  A[_20] = sa * st;
  A[_30] = 0.0;
  A[_01] = -st;
  A[_11] = ca * ct;
  A[_21] = sa * ct;
  A[_31] = 0.0;
  A[_02] = 0.0;
  A[_12] = -sa;
  A[_22] = ca;
  A[_32] = 0.0;
  A[_03] = a;
  A[_13] = -sa * d;
  A[_23] = ca * d;
  A[_33] = 1.0;

}

void srvlib::davinci::extendChain(const DenavitHartenbergFrame& frame, GLfloat* A, float delta = 0.0) {

  GLfloat DH[16];

  switch (frame.mJointType)
  {
  case JointTypeEnum::FIXED:
    glhDenavitHartenberg(frame.mA * SCALE, frame.mAlpha, frame.mD * SCALE, frame.mTheta, DH);
    break;

  case JointTypeEnum::ROTARY:
    glhDenavitHartenberg(frame.mA * SCALE, frame.mAlpha, frame.mD * SCALE, frame.mTheta + delta, DH);
    break;

  case JointTypeEnum::PRISMATIC:
    glhDenavitHartenberg(frame.mA * SCALE, frame.mAlpha, (frame.mD + delta) * SCALE, frame.mTheta, DH);
    break;
  }

  glhMultMatrixRight(DH, A);

}

// Concatenate chain transformations
void srvlib::davinci::buildKinematicChainPSM2(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2) {

  GLfloat A[16];
  glhSetIdentity(A);

  extendChain(mDaVinciChain.mWorldOriginSUJ2Origin[0], A);
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[0], A, psm.sj_joint_angles[0]);
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[1], A, psm.sj_joint_angles[1]);
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[2], A, psm.sj_joint_angles[2]);
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[3], A, psm.sj_joint_angles[3]);
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[4], A, psm.sj_joint_angles[4]);
  extendChain(mDaVinciChain.mSUJ2OriginSUJ2Tip[5], A, psm.sj_joint_angles[5]);

  extendChain(mDaVinciChain.mSUJ2TipPSM2Origin[0], A);
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[0], A, psm.jnt_pos[0]);
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[1], A, psm.jnt_pos[1]);
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[2], A, psm.jnt_pos[2]);

  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[3], A, psm.jnt_pos[3]);
  roll = glm::make_mat4(A);

  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[4], A, psm.jnt_pos[4]);

  wrist_pitch = glm::make_mat4(A);

  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[5], A, psm.jnt_pos[5]);

  glm::mat4 wrist_yaw = glm::make_mat4(A);

  //no dh param here as this just points in the direction of the instrument head
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[6], A, 0);

  //rotate the instrument claspers around the clasper axis 
  grip1 = glm::make_mat4(A);
  grip2 = glm::make_mat4(A);

  //this is the angle between the claspers, so each clasper rotates 0.5*angle away from the center point
  float clasper_angle = psm.jnt_pos[6];

  glm::mat4 clasper_rotation;
  clasper_rotation = glm::rotate((float)(0.5 * clasper_angle), glm::vec3(0, 1, 0));

  glm::mat4 grip1d = grip1;
  glm::mat4 grip2d = grip2;

  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip1d));
  clasper_rotation = glm::rotate((float)(-0.5*clasper_angle), glm::vec3(0, 1, 0));
  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip2d));

  grip1 = grip1d;
  grip2 = grip2d;

}

void srvlib::davinci::buildKinematicChainAtEndPSM1(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, const glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2){

  //glm::mat4 A = roll;
  GLfloat A[16];
  memcpy(A, &roll[0][0], sizeof(float) * 16);

  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[4], A, psm.jnt_pos[0]);
  wrist_pitch = glm::make_mat4(A);

  //don't actually care about wrist yaw as the clasper pose 'contains' this information
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[5], A, psm.jnt_pos[1]);
  glm::mat4 wrist_yaw = glm::make_mat4(A);

  //transform into the clasper reference frame
  //no dh param here as this just points in the direction of the instrument head
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[6], A, 0);

  //rotate the instrument claspers around the clasper axis 
  grip1 = glm::make_mat4(A);
  grip2 = glm::make_mat4(A);

  //this is the angle between the claspers, so each clasper rotates 0.5*angle away from the center point
  float clasper_angle = psm.jnt_pos[2];

  glm::mat4 clasper_rotation = glm::rotate(clasper_angle, glm::vec3(0, 1, 0));

  glm::mat4 grip1d = grip1;
  glm::mat4 grip2d = grip2;

  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip1d));
  clasper_rotation = glm::rotate(-clasper_angle, glm::vec3(0, 1, 0));
  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip2d));

  grip1 = grip1d;
  grip2 = grip2d;

}

void srvlib::davinci::buildKinematicChainECM1(DaVinciKinematicChain &mDaVinciChain, const ECMData& ecm, glm::mat4 &world_to_camera_transform) {

  GLfloat A[16];
  glhSetIdentity(A);

  extendChain(mDaVinciChain.mWorldOriginSUJ3Origin[0], A);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[0], A, ecm.sj_joint_angles[0]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[1], A, ecm.sj_joint_angles[1]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[2], A, ecm.sj_joint_angles[2]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[3], A, ecm.sj_joint_angles[3]);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[4], A);
  extendChain(mDaVinciChain.mSUJ3OriginSUJ3Tip[5], A);
  extendChain(mDaVinciChain.mSUJ3TipECM1Origin[0], A);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[0], A, ecm.jnt_pos[0]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[1], A, ecm.jnt_pos[1]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[2], A, ecm.jnt_pos[2]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[3], A, ecm.jnt_pos[3]);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[4], A);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[5], A);
  extendChain(mDaVinciChain.mECM1OriginECM1Tip[6], A);

  world_to_camera_transform = glm::make_mat4(A);

}

void srvlib::davinci::buildKinematicChainAtEndPSM2(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, const glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2){

  GLfloat A[16];
  memcpy(A, &roll[0][0], sizeof(float) * 16);

  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[4], A, psm.jnt_pos[0]);
  wrist_pitch = glm::make_mat4(A);

  //don't actually care about wrist yaw as the clasper pose 'contains' this information
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[5], A, psm.jnt_pos[1]);
  glm::mat4 wrist_yaw = glm::make_mat4(A);

  //transform into the clasper reference frame
  //no dh param here as this just points in the direction of the instrument head
  extendChain(mDaVinciChain.mPSM2OriginPSM2Tip[6], A, 0);

  //rotate the instrument claspers around the clasper axis 
  grip1 = glm::make_mat4(A);
  grip2 = glm::make_mat4(A);

  //this is the angle between the claspers, so each clasper rotates 0.5*angle away from the center point
  float clasper_angle = psm.jnt_pos[2];
  glm::mat4 clasper_rotation = glm::rotate(clasper_angle, glm::vec3(0, 1, 0));
  glm::mat4 grip1d = grip1;
  glm::mat4 grip2d = grip2;

  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip1d));
  clasper_rotation = glm::rotate(-clasper_angle, glm::vec3(0, 1, 0));
  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip2d));

  grip1 = grip1d;
  grip2 = grip2d;

}

void srvlib::davinci::buildKinematicChainPSM1(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2){

  GLfloat A[16];
  glhSetIdentity(A);

  extendChain(mDaVinciChain.mWorldOriginSUJ1Origin[0], A);
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[0], A, psm.sj_joint_angles[0]);
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[1], A, psm.sj_joint_angles[1]);
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[2], A, psm.sj_joint_angles[2]);
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[3], A, psm.sj_joint_angles[3]);
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[4], A, psm.sj_joint_angles[4]);
  extendChain(mDaVinciChain.mSUJ1OriginSUJ1Tip[5], A, psm.sj_joint_angles[5]);

  extendChain(mDaVinciChain.mSUJ1TipPSM1Origin[0], A);
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[0], A, psm.jnt_pos[0]);
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[1], A, psm.jnt_pos[1]);
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[2], A, psm.jnt_pos[2]);

  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[3], A, psm.jnt_pos[3]);
  roll = glm::make_mat4(A);

  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[4], A, psm.jnt_pos[4]);
  wrist_pitch = glm::make_mat4(A);

  //don't actually care about wrist yaw as the clasper pose 'contains' this information
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[5], A, psm.jnt_pos[5]);
  glm::mat4 wrist_yaw = glm::make_mat4(A);

  //transform into the clasper reference frame
  //no dh param here as this just points in the direction of the instrument head
  extendChain(mDaVinciChain.mPSM1OriginPSM1Tip[6], A, 0);

  //rotate the instrument claspers around the clasper axis 
  grip1 = glm::make_mat4(A);
  grip2 = glm::make_mat4(A);

  //this is the angle between the claspers, so each clasper rotates 0.5*angle away from the center point
  float clasper_angle = psm.jnt_pos[6];

  glm::mat4 clasper_rotation = glm::rotate(0.5f*clasper_angle, glm::vec3(0, 1, 0));
  glm::mat4 grip1d = grip1;
  glm::mat4 grip2d = grip2;

  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip1d));
  clasper_rotation = glm::rotate(-0.5f*clasper_angle, glm::vec3(0, 1, 0));
  glhMultMatrixRight(glm::value_ptr(clasper_rotation), glm::value_ptr(grip2d));

  grip1 = grip1d;
  grip2 = grip2d;




}