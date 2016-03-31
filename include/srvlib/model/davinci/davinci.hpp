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

This file was based on work by Philip Pratt, Imperial College London. Used with permission.

**/
#include <glm/matrix.hpp>
#include <vector>

#include <srvlib/model/model.hpp>



#define _00 0
#define _10 1
#define _20 2
#define _30 3
#define _01 4
#define _11 5
#define _21 6
#define _31 7
#define _02 8
#define _12 9
#define _22 10
#define _32 11
#define _03 12  
#define _13 13
#define _23 14
#define _33 15

typedef float GLfloat;

namespace srvlib {

  namespace davinci {


    class DaVinciInstrument : public BaseModel {

    public:

      virtual void Draw() const;

      void DrawBody() const;
      void DrawLeftClasper() const;
      void DrawRightClasper() const;
      void DrawHead() const;


      virtual void LoadData(const std::string &datafile_path);

      virtual std::vector<glm::mat4> GetTransformSet() const;
      virtual void SetTransformSet(const std::vector<glm::mat4> &transforms);

      void UpdatePose(const glm::vec4 &quaternion_update, const glm::vec3 &translation_update, const glm::vec3 &articulation_update);
      void EditPose(const glm::quat &rotation, const glm::vec3 &translation, const glm::vec3 &articulation);

      void UpdatePose(const glm::vec3 &euler_update, const std::string &euler_order, const bool use_intrinsic_eulers, const glm::vec3 &translation_update, const glm::vec3 &articulation_update);
      void EditPose(const glm::vec3 &euler_rotation, const std::string &euler_order, const bool use_intrinsic_eulers, const glm::vec3 &translation, const glm::vec3 &articulation);

      RenderData &Shaft() { return shaft_; }
      RenderData &Head() { return head_; }
      RenderData &Clasper1() { return clasper1_; }
      RenderData &Clasper2() { return clasper2_; }

      const RenderData &Shaft() const { return shaft_; }
      const RenderData &Head() const { return head_; }
      const RenderData &Clasper1() const { return clasper1_; }
      const RenderData &Clasper2() const { return clasper2_; }

    protected:

      RenderData shaft_;
      RenderData head_;
      RenderData clasper1_;
      RenderData clasper2_;

    };


    /**
    * @struct PSMData
    * Basic wrapper for the data that comes from a Da Vinci PSM
    */
    struct PSMData {

      float jnt_pos[7]; /**< The arm joint positions. */
      float sj_joint_angles[6]; /**< The set up joint positions. */

    };

    /**
    * @struct ECMData
    * Basic wrapper for the data that comes from a Da Vinci ECM
    */
    struct ECMData {

      float jnt_pos[4]; /**< The arm joint positions. */
      float sj_joint_angles[6];  /**< The set up joint positions. */

    };

    /**
    * @enum DaVinciJoint
    * The arms on a classic da Vinci.
    */
    enum DaVinciJoint { PSM1, PSM2, PSM3, ECM };

    /**
    * @enum JointTypeEnum
    * The types of joint on a da Vinci arm.
    */
    struct JointTypeEnum {
      enum Enum {
        FIXED = 0,
        ROTARY = 1,
        PRISMATIC = 2,
      };
    };

    /**
    * @struct GeneralFrame
    * The general 3D transformation type for a rigid body.
    */
    struct GeneralFrame{

      /**
      * Default constructor. Sets the transform to null translation and identity rotation.
      */
      GeneralFrame() : mX(0.0f), mY(0.0f), mZ(0.0f), mR00(1.0f), mR01(0.0f), mR02(0.0f), mR10(0.0f), mR11(1.0f), mR12(0.0f), mR20(0.0f), mR21(0.0f), mR22(1.0f) {}

      /**
      * Construct a 3D rigid body transform.
      * @param[in] x The x translation component.
      * @param[in] y The y translation component.
      * @param[in] z The z translation component.
      * @param[in] r00 The row = 1, col = 1 rotation component.
      * @param[in] r01 The row = 1, col = 2 rotation component.
      * @param[in] r02 The row = 1, col = 3 rotation component.
      * @param[in] r10 The row = 2, col = 1 rotation component.
      * @param[in] r11 The row = 2, col = 2 rotation component.
      * @param[in] r12 The row = 2, col = 3 rotation component.
      * @param[in] r20 The row = 3, col = 1 rotation component.
      * @param[in] r21 The row = 3, col = 2 rotation component.
      * @param[in] r22 The row = 3, col = 3 rotation component.
      */
      GeneralFrame(float x, float y, float z, float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22) :
        mX(x), mY(y), mZ(z), mR00(r00), mR01(r01), mR02(r02), mR10(r10), mR11(r11), mR12(r12), mR20(r20), mR21(r21), mR22(r22) {}

      float mX; /**< The x translation component. */
      float mY; /**< The y translation component. */
      float mZ; /**< The z translation component. */
      float mR00; /**< The row = 1, col = 1 rotation component. */
      float mR01; /**< The row = 1, col = 2 rotation component. */
      float mR02; /**< The row = 1, col = 3 rotation component. */
      float mR10; /**< The row = 2, col = 1 rotation component. */
      float mR11; /**< The row = 2, col = 2 rotation component. */
      float mR12; /**< The row = 2, col = 3 rotation component. */
      float mR20; /**< The row = 3, col = 1 rotation component. */
      float mR21; /**< The row = 3, col = 2 rotation component. */
      float mR22; /**< The row = 3, col = 3 rotation component. */

    };

    /**
    * @struct DenavitHartenbergFrame
    * The reference frame transformation for a kinematic chain when using the Denavit Hartenberg parameters.
    */
    struct DenavitHartenbergFrame{

      /**
      * Default constructor. Sets a, &alpha, d, &theta to zero.
      */
      DenavitHartenbergFrame() : mFrame(0), mJointType(JointTypeEnum::FIXED), mA(0.0f), mAlpha(0.0f), mD(0.0f), mTheta(0.0f){}

      /**
      * Sets up a Denavit Hartenberg Frame.
      * @param[in] frame The frame index.
      * @param[in] joint_type Whether the joint is rotary, prismatic or fixed.
      * @param[in] a The value of a.
      * @param[in] alpha The value of &alpha.
      * @param[in] d The value of d.
      * @param[in] theta The value of &theta.
      */
      DenavitHartenbergFrame(unsigned int frame, JointTypeEnum::Enum joint_type, float a, float alpha, float d, float theta) :
        mFrame(frame), mJointType(joint_type), mA(a), mAlpha(alpha), mD(d), mTheta(theta){ }

      unsigned int mFrame; /**< The frame index. */
      JointTypeEnum::Enum mJointType; /**< The joint type. */
      float mA; /**< The value of a. */
      float mAlpha; /**< The value of &alpha. */
      float mD; /**< The value of d. */
      float mTheta; /**< The value of &theta. */

    };

    /**
    * @struct DaVinciKinematicChain
    * A kinematic chain representing a da Vinci classic robot with 2 PSMs and 1 ECM.
    */
    struct DaVinciKinematicChain {

      /**
      * Sets up the arm parameters and builds the kinematic chain for a classic da Vinci (as these are constant).
      */
      DaVinciKinematicChain();

      // PSM1
      std::vector<GeneralFrame> mWorldOriginSUJ1Origin; /**< The set of transforms between the robot origin and the start of the set up joints on PSM1. */
      std::vector<DenavitHartenbergFrame> mSUJ1OriginSUJ1Tip;  /**< The set of transforms between the start of the set up joints and the end of the set up joints on PSM1. */
      std::vector<GeneralFrame> mSUJ1TipPSM1Origin;  /**< The set of transforms between the end of the set up joints and the start of the arm joints on PSM1. */
      std::vector<DenavitHartenbergFrame> mPSM1OriginPSM1Tip;  /**< The set of transforms between the start of the arm joints and the end of the arm joints on PSM1. */

      // PSM2
      std::vector<GeneralFrame> mWorldOriginSUJ2Origin;  /**< The set of transforms between the robot origin and the start of the set up joints on PSM2. */
      std::vector<DenavitHartenbergFrame> mSUJ2OriginSUJ2Tip;  /**< The set of transforms between the start of the set up joints and the end of the set up joints on PSM2. */
      std::vector<GeneralFrame> mSUJ2TipPSM2Origin;  /**< The set of transforms between the end of the set up joints and the start of the arm joints on PSM2. */
      std::vector<DenavitHartenbergFrame> mPSM2OriginPSM2Tip; /**< The set of transforms between the start of the arm joints and the end of the arm joints on PSM2. */

      // ECM1
      std::vector<GeneralFrame> mWorldOriginSUJ3Origin; /**< The set of transforms between the robot origin and the start of the set up joints on ECM1. */
      std::vector<DenavitHartenbergFrame> mSUJ3OriginSUJ3Tip;  /**< The set of transforms between the start of the set up joints and the end of the set up joints on ECM1. */
      std::vector<GeneralFrame> mSUJ3TipECM1Origin;  /**< The set of transforms between the end of the set up joints and the start of the arm joints on ECM1. */
      std::vector<DenavitHartenbergFrame> mECM1OriginECM1Tip; /**< The set of transforms between the start of the arm joints and the end of the arm joints on ECM1. */

    };

    /**
    * Helper function to initialise a 4x4 rigid body transform to the indentity matrix. Uses OpenGL format so matrix is column major.
    * @param[out] The identity matrix.
    */
    template<typename T>
    void glhSetIdentity(T* A){
      A[_00] = (T)1.0; A[_01] = (T)0.0; A[_02] = (T)0.0; A[_03] = (T)0.0;
      A[_10] = (T)0.0; A[_11] = (T)1.0; A[_12] = (T)0.0; A[_13] = (T)0.0;
      A[_20] = (T)0.0; A[_21] = (T)0.0; A[_22] = (T)1.0; A[_23] = (T)0.0;
      A[_30] = (T)0.0; A[_31] = (T)0.0; A[_32] = (T)0.0; A[_33] = (T)1.0;

    }


    /**
    * Helper function to multiply two matrices in the order BA. Uses OpenGL format so matrix is column major.
    * @param[in] A The right matrix.
    * @param[in,out] B The left matrix. Also stores the result.
    */

    // Matrix multiplication (B = BA)
    template<typename T>
    void glhMultMatrixRight(const T* A, T* B){

      T M[16];
      M[_00] = B[_00] * A[_00] + B[_01] * A[_10] + B[_02] * A[_20] + B[_03] * A[_30];
      M[_10] = B[_10] * A[_00] + B[_11] * A[_10] + B[_12] * A[_20] + B[_13] * A[_30];
      M[_20] = B[_20] * A[_00] + B[_21] * A[_10] + B[_22] * A[_20] + B[_23] * A[_30];
      M[_30] = B[_30] * A[_00] + B[_31] * A[_10] + B[_32] * A[_20] + B[_33] * A[_30];
      M[_01] = B[_00] * A[_01] + B[_01] * A[_11] + B[_02] * A[_21] + B[_03] * A[_31];
      M[_11] = B[_10] * A[_01] + B[_11] * A[_11] + B[_12] * A[_21] + B[_13] * A[_31];
      M[_21] = B[_20] * A[_01] + B[_21] * A[_11] + B[_22] * A[_21] + B[_23] * A[_31];
      M[_31] = B[_30] * A[_01] + B[_31] * A[_11] + B[_32] * A[_21] + B[_33] * A[_31];
      M[_02] = B[_00] * A[_02] + B[_01] * A[_12] + B[_02] * A[_22] + B[_03] * A[_32];
      M[_12] = B[_10] * A[_02] + B[_11] * A[_12] + B[_12] * A[_22] + B[_13] * A[_32];
      M[_22] = B[_20] * A[_02] + B[_21] * A[_12] + B[_22] * A[_22] + B[_23] * A[_32];
      M[_32] = B[_30] * A[_02] + B[_31] * A[_12] + B[_32] * A[_22] + B[_33] * A[_32];
      M[_03] = B[_00] * A[_03] + B[_01] * A[_13] + B[_02] * A[_23] + B[_03] * A[_33];
      M[_13] = B[_10] * A[_03] + B[_11] * A[_13] + B[_12] * A[_23] + B[_13] * A[_33];
      M[_23] = B[_20] * A[_03] + B[_21] * A[_13] + B[_22] * A[_23] + B[_23] * A[_33];
      M[_33] = B[_30] * A[_03] + B[_31] * A[_13] + B[_32] * A[_23] + B[_33] * A[_33];

      for (unsigned int i = 0; i < 16; i++)
        B[i] = M[i];

    }

    /**
    * Transform a reference frame A on a rigid body by a GeneralFrame transform as A = A*frame
    * @param[in] frame The rigid body reference frame transform.
    * @param[in,out] A The current reference frame which is updated by frame.
    */
    void extendChain(const GeneralFrame& frame, GLfloat* A);

    /**
    * Construct a rigid body transform from the DH parameters. This is using the modified DH paramter representation.
    * @param[in] a The length of the common normal between the links that the transform describes.
    * @param[in] alpha The angle about the common normal between the links that transform describes.
    * @param[in] d The angle about the common normal between the links that transform describes.
    * @param[in] theta The angle about the common normal between the links that transform describes.
    * @param[out] A The output transformation matrix.
    */
    void glhDenavitHartenberg(const GLfloat a, const GLfloat alpha, const GLfloat d, const GLfloat theta, GLfloat *A);

    /**
    * Transform a reference frame A on a rigid body by a DenavitHartenberg transform as A = A*frame. The DH frame has moved from its 'home' position which we specified in the DH parameters when we constructed it to a new position by rotation or translating by the amount @delta.
    * @param[in] frame The rigid body reference frame transform.
    * @param[in,out] A The current reference frame which is updated by frame.
    * @param[in] delta The change to the rotatry or translation joint.
    */
    void extendChain(const DenavitHartenbergFrame& frame, GLfloat* A, float delta);

    /**
    * Build the kinematic chain for PSM1 at the current pose.
    * Gives the useful transforms from the robot world coordinates to the coordinates of the instrument at the end of the arm.
    * @param[in] mDaVinciChain The kinematic chain representing the PSM1.
    * @param[in] psm The current pose information describing the position of the PSM.
    * @param[out] roll The transform from the robot world coordinates to the instrument roll axis.
    * @param[out] wrist_pitch The transform from the robot world coordinates to the instrument wrist coordinates.
    * @param[out] grip1 The transform from the robot world coordinates to first instrument grip. Not really sure how to describe 'which' of the 2 grips this is...
    * @param[out] grip2 The transform from the robot world coordinates to second instrument grip.
    */
    void buildKinematicChainPSM1(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2);

    /**
    * Build the kinematic chain for PSM1 when we know the transform from camera to roll coordinates (for example because we tracked its pose using a vision method).
    * Gives the useful transforms from the robot world coordinates to the coordinates of the instrument at the end of the arm.
    * @param[in] mDaVinciChain The kinematic chain representing the PSM1.
    * @param[in] psm The current pose information describing the position of the PSM.
    * @param[in] roll The transform from the robot world coordinates to the instrument roll axis.
    * @param[out] wrist_pitch The transform from the robot world coordinates to the instrument wrist coordinates.
    * @param[out] grip1 The transform from the robot world coordinates to first instrument grip. Not really sure how to describe 'which' of the 2 grips this is...
    * @param[out] grip2 The transform from the robot world coordinates to second instrument grip.
    */
    void buildKinematicChainAtEndPSM1(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, const glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2);

    /**
    * Build the kinematic chain for PSM2 at the current pose.
    * Gives the useful transforms from the robot world coordinates to the coordinates of the instrument at the end of the arm.
    * @param[in] mDaVinciChain The kinematic chain representing the PSM2.
    * @param[in] psm The current pose information describing the position of the PSM.
    * @param[out] roll The transform from the robot world coordinates to the instrument roll axis.
    * @param[out] wrist_pitch The transform from the robot world coordinates to the instrument wrist coordinates.
    * @param[out] grip1 The transform from the robot world coordinates to first instrument grip. Not really sure how to describe 'which' of the 2 grips this is...
    * @param[out] grip2 The transform from the robot world coordinates to second instrument grip.
    */
    void buildKinematicChainPSM2(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2);

    /**
    * Build the kinematic chain for PSM2 when we know the transform from camera to roll coordinates (for example because we tracked its pose using a vision method).
    * Gives the useful transforms from the robot world coordinates to the coordinates of the instrument at the end of the arm.
    * @param[in] mDaVinciChain The kinematic chain representing the PSM2.
    * @param[in] psm The current pose information describing the position of the PSM.
    * @param[in] roll The transform from the robot world coordinates to the instrument roll axis.
    * @param[out] wrist_pitch The transform from the robot world coordinates to the instrument wrist coordinates.
    * @param[out] grip1 The transform from the robot world coordinates to first instrument grip. Not really sure how to describe 'which' of the 2 grips this is...
    * @param[out] grip2 The transform from the robot world coordinates to second instrument grip.
    */
    void buildKinematicChainAtEndPSM2(DaVinciKinematicChain &mDaVinciChain, const PSMData& psm, const glm::mat4 &roll, glm::mat4 &wrist_pitch, glm::mat4 &grip1, glm::mat4 &grip2);

    /**
    * Build the kinematic chain for ECM at the current pose.
    * Gives the useful transforms from the robot world coordinates to the coordinates of the camera at the end of the arm.
    * @param[in] mDaVinciChain The kinematic chain representing the ECM.
    * @param[in] ecm The current pose information describing the position of the ECM.
    * @param[out] world_to_camera_transform The transform from the robot world coordinates to the camera reference frame.
    */
    void buildKinematicChainECM1(DaVinciKinematicChain &mDaVinciChain, const ECMData& ecm, glm::mat4 &world_to_camera_transform);

  }
}