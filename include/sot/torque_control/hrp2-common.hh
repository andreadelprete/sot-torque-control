/*
 * Copyright 2014, Oscar E. Ramos Ponce, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-torque-control is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-torque-control is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-torque-control.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __sot_torque_control_hrp2_common_H__
#define __sot_torque_control_hrp2_common_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (hrp2_common_EXPORTS)
#    define HRP2COMMON_EXPORT __declspec(dllexport)
#  else
#    define HRP2COMMON_EXPORT __declspec(dllimport)
#  endif
#else
#  define HRP2COMMON_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"


namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

#define N_JOINTS 30

const double DEFAULT_MAX_DELTA_Q = 0.1; /// max joint position tracking error [rad]

const double DEFAULT_MAX_CURRENT = 5;     /// max CURRENT (double in [0 Amp, 20 Amp]) 

      // Information on the location of the IMU and F/T sensors of HRP-2
      // copied from the urdf file (in stacks/hrp2/hrp2_14_description/urdf):
      //      <joint name="AccelerometerJoint" type="fixed">
      //        <origin xyz="-0.13 0.0 0.118" rpy="0.0 0.0 0.0"/>
      //        <joint name="RightFootForceSensorJoint" type="fixed">
      //          <origin xyz="0.0 0.0 -0.105" rpy="0.0 0.0 0.0"/>
      //        <joint name="LeftFootForceSensorJoint" type="fixed">
      //          <origin xyz="0.0 0.0 -0.105" rpy="0.0 0.0 0.0"/>
      //        <joint name="RightHandForceSensorJoint" type="fixed">
      //          <origin xyz="0.005 0.0 -0.05925" rpy="0.0 0.0 0.0"/>
      //        <joint name="LeftHandForceSensorJoint" type="fixed">
      //          <origin xyz="0.005 0.0 -0.05925" rpy="0.0 0.0 0.0"/>

      /// Position of the IMU w.r.t. the frame of the hosting link (torso)
      const double IMU_XYZ[3]                     = {-0.13, 0.0,  0.118};

      /// Position of the force/torque sensors w.r.t. the frame of the hosting link
      const double RIGHT_FOOT_FORCE_SENSOR_XYZ[3] = {0.0,   0.0, -0.085};
      const double LEFT_FOOT_FORCE_SENSOR_XYZ[3]  = {0.0,   0.0, -0.085};
      const double RIGHT_HAND_FORCE_SENSOR_XYZ[3] = {0.005, 0.0, -0.05925};
      const double LEFT_HAND_FORCE_SENSOR_XYZ[3]  = {0.005, 0.0, -0.05925};

      /// Rotation angle around Z axis of the force/torque sensors w.r.t. the frame of the hosting link
      const double RIGHT_HAND_FORCE_SENSOR_Z_ROTATION = -0.5*M_PI;
      const double LEFT_HAND_FORCE_SENSOR_Z_ROTATION  = -0.5*M_PI;

      /// Position of the foot soles w.r.t. the frame of the foot
      const double RIGHT_FOOT_SOLE_XYZ[3]         = {0.0,   0.0, -0.105};
      const double LEFT_FOOT_SOLE_XYZ[3]          = {0.0,   0.0, -0.105};

      /// Position of the hand grippers w.r.t. the frame of the hand
      const double RIGHT_HAND_GRIPPER_XYZ[3]         = {0.0,   0.0, 0.0};
      const double LEFT_HAND_GRIPPER_XYZ[3]          = {0.0,   0.0, 0.0};

      /// Percentage of mass of the link that is measured by the F/T sensors
      const double RIGHT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE = 0.65;
      const double LEFT_FOOT_FORCE_SENSOR_MASS_PERCENTAGE  = 0.65;
      const double RIGHT_HAND_FORCE_SENSOR_MASS_PERCENTAGE = 0.75;
      const double LEFT_HAND_FORCE_SENSOR_MASS_PERCENTAGE  = 0.75;

      struct JointLimits
      {
        double upper;
        double lower;

        JointLimits():
          upper(0.0),
          lower(0.0)
        {}

        JointLimits(double l, double u):
          upper(u),
          lower(l)
        {}
      };

      /// Map from joint names to joint ids
      struct JointUtil
      {
        static std::map<unsigned int,JointLimits> create_id_2_limits_map()
        {
          std::map<unsigned int,JointLimits> m;
          m[0] = JointLimits(-0.785398, 0.523599);  // right hip yaw
          m[1] = JointLimits(-0.610865, 0.349066);  // right hip roll
          m[2] = JointLimits(-2.18166, 0.733038);   // right hip pitch
          m[3]  = JointLimits(-0.0349066, 2.61799); // right knee
          m[4] = JointLimits(-1.309, 0.733038);     // right ankle pitch
          m[5] = JointLimits(-0.349066, 0.610865);  // right ankle roll
          m[6] = JointLimits(-0.523599, 0.785398);  // left hip yaw
          m[7] = JointLimits(-0.349066, 0.610865);  // left hip roll
          m[8] = JointLimits(-2.18166, 0.733038);   // left hip pitch
          m[9]  = JointLimits(-0.0349066, 2.61799); // left knee
          m[10] = JointLimits(-1.309, 0.733038);    // left ankle pitch
          m[11] = JointLimits(-0.610865, 0.349066); // left ankle roll
          m[12]  = JointLimits(-0.785398, 0.785398);// torso yaw
          m[13]  = JointLimits(-0.0872665, 1.0472); // torso pitch
          m[14]  = JointLimits(-0.785398, 0.78539); // head yaw
          m[15]  = JointLimits(-0.523599, 0.785398);// head pitch
          m[16] = JointLimits(-3.14159, 1.0472);    // right shoulder pitch
          m[17] = JointLimits(-1.65806, 0.174533);  // right shoulder roll
          m[18] = JointLimits(-1.6057, 1.6057);     // right shoulder yaw
          m[19] = JointLimits(-2.3911, 0.0349066);  // right elbow
          m[20] = JointLimits(-1.6057, 1.6057);     // right wrist yaw
          m[21]  = JointLimits(-1.6057, 1.6057);    // right wrist pitch
          m[22]  = JointLimits(-1.0, 1.0);          // right hand
          m[23] = JointLimits(-3.14159, 1.0472);    // left shoulder pitch
          m[24] = JointLimits(-0.174533, 1.65806);  // left shoulder roll
          m[25] = JointLimits(-1.6057, 1.6057);     // left shoulder yaw
          m[26] = JointLimits(-2.3911, 0.0349066);  // left elbow
          m[27] = JointLimits(-1.6057, 1.6057);     // left wrist yaw
          m[28]  = JointLimits(-1.6057, 1.6057);    // left wrist pitch
          m[29]  = JointLimits(-1.0, 1.0);          // left hand
          return m;
        }

        static std::map<std::string,unsigned int> create_name_2_id_map()
        {
          std::map<std::string,unsigned int> m;
          m["rhy"] = 0;   // right hip yaw
          m["rhr"] = 1;   // right hip roll
          m["rhp"] = 2;   // right hip pitch
          m["rk"]  = 3;   // right knee
          m["rap"] = 4;   // right ankle pitch
          m["rar"] = 5;   // right ankle roll
          m["lhy"] = 6;   // left hip yaw
          m["lhr"] = 7;   // left hip roll
          m["lhp"] = 8;   // left hip pitch
          m["lk"]  = 9;   // left knee
          m["lap"] = 10;  // left ankle pitch
          m["lar"] = 11;  // left ankle roll
          m["ty"]  = 12;  // torso yaw
          m["tp"]  = 13;  // torso pitch
          m["hy"]  = 14;  // head yaw
          m["hp"]  = 15;  // head pitch
          m["rsp"] = 16;  // right shoulder pitch
          m["rsr"] = 17;  // right shoulder roll
          m["rsy"] = 18;  // right shoulder yaw
          m["re"]  = 19;  // right elbow
          m["rwy"] = 20;  // right wrist yaw
          m["rwp"] = 21;  // right wrist pitch
          m["rh"]  = 22;  // right hand
          m["lsp"] = 23;  // left shoulder pitch
          m["lsr"] = 24;  // left shoulder roll
          m["lsy"] = 25;  // left shoulder yaw
          m["le"]  = 26;  // left elbow
          m["lwy"] = 27;  // left wrist yaw
          m["lwp"] = 28;  // left wrist pitch
          m["lh"]  = 29;  // left hand
          return m;
        }

        static std::map<unsigned int,std::string> create_id_2_name_map(
            const std::map<std::string,unsigned int>& name_2_id_map)
        {
          std::map<unsigned int,std::string> m;
          std::map<std::string, unsigned int>::const_iterator it;
          for(it = name_2_id_map.begin(); it != name_2_id_map.end(); it++)
            m[it->second] = it->first;
          return m;
        }

        /** Given a joint name it finds the associated joint id.
         * If the specified joint name is not found it returns -1;
         * @param name Name of the joint to find.
         * @return The id of the specified joint, -1 if not found. */
        static int get_id_from_name(std::string name)
        {
          std::map<std::string,unsigned int>::const_iterator iter = name_2_id.find(name);
          if (iter==name_2_id.end())
            return -1;
          return iter->second;
        }

        /** Given a joint id it finds the associated joint name.
         * If the specified joint is not found it returns "Joint name not found";
         * @param id Id of the joint to find.
         * @return The name of the specified joint, "Joint name not found" if not found. */
        static std::string get_name_from_id(unsigned int id)
        {
          std::map<unsigned int,std::string>::const_iterator iter = id_2_name.find(id);
          if(iter==id_2_name.end())
            return "Joint name not found";
          return iter->second;
        }

        /** Given a joint id it finds the associated joint limits.
         * If the specified joint is not found it returns JointLimits(0,0).
         * @param id Id of the joint to find.
         * @return The limits of the specified joint, JointLimits(0,0) if not found. */
        static JointLimits get_limits_from_id(unsigned int id)
        {
          std::map<unsigned int,JointLimits>::const_iterator iter = id_2_limits.find(id);
          if(iter==id_2_limits.end())
            return JointLimits(0.0,0.0);
          return iter->second;
        }

        static const std::map<std::string,unsigned int> name_2_id;
        static const std::map<unsigned int,std::string> id_2_name;
        static const std::map<unsigned int,JointLimits> id_2_limits;
      };
      const std::map<std::string,unsigned int> JointUtil::name_2_id = JointUtil::create_name_2_id_map();
      const std::map<unsigned int,std::string> JointUtil::id_2_name = JointUtil::create_id_2_name_map(JointUtil::name_2_id);
      const std::map<unsigned int,JointLimits> JointUtil::id_2_limits = JointUtil::create_id_2_limits_map();



      struct ForceLimits
      {
        Eigen::VectorXd upper;
        Eigen::VectorXd lower;

        ForceLimits():
          upper(Eigen::Vector6d::Zero()),
          lower(Eigen::Vector6d::Zero())
        {}

        ForceLimits(const Eigen::VectorXd& l, const Eigen::VectorXd& u):
          upper(u),
          lower(l)
        {}
      };

      enum ForceID
      {
        FORCE_ID_RIGHT_FOOT = 0,
        FORCE_ID_LEFT_FOOT  = 1,
        FORCE_ID_RIGHT_HAND = 2,
        FORCE_ID_LEFT_HAND  = 3
      };

      /// Map from force names to force ids
      struct ForceUtil
      {
        static std::map<unsigned int,ForceLimits> create_id_2_limits_map()
        {
          Eigen::Vector6d fMax, fMin;
          fMax << 100.0, 100.0, 300.0, 80.0, 80.0, 30.0;
          fMin = -fMax;
          std::map<unsigned int,ForceLimits> m;
          m[0] = ForceLimits(fMin, fMax);  // right foot
          m[1] = ForceLimits(fMin, fMax);  // left foot
          m[2] = ForceLimits(fMin, fMax);   // right hand
          m[3] = ForceLimits(fMin, fMax); // left hand
          return m;
        }

        static std::map<std::string,unsigned int> create_name_2_id_map()
        {
          std::map<std::string,unsigned int> m;
          m["rf"] = 0;   // right foot
          m["lf"] = 1;   // left foot
          m["rh"] = 2;   // right hand
          m["lh"] = 3;   // left hand
          return m;
        }

        static std::map<unsigned int,std::string> create_id_2_name_map(
            const std::map<std::string,unsigned int>& name_2_id_map)
        {
          std::map<unsigned int,std::string> m;
          std::map<std::string, unsigned int>::const_iterator it;
          for(it = name_2_id_map.begin(); it != name_2_id_map.end(); it++)
            m[it->second] = it->first;
          return m;
        }

        /** Given a force name it finds the associated id.
         * If the specified force name is not found it returns -1;
         * @param name Name of the force to find.
         * @return The id of the specified force, -1 if not found. */
        static int get_id_from_name(std::string name)
        {
          std::map<std::string,unsigned int>::const_iterator iter = name_2_id.find(name);
          if (iter==name_2_id.end())
            return -1;
          return iter->second;
        }

        /** Given a force id it finds the associated name.
         * If the specified force is not found it returns "Force name not found";
         * @param id Id of the force to find.
         * @return The name of the specified force, "Force name not found" if not found. */
        static std::string get_name_from_id(unsigned int id)
        {
          std::map<unsigned int,std::string>::const_iterator iter = id_2_name.find(id);
          if(iter==id_2_name.end())
            return "Force name not found";
          return iter->second;
        }

        /** Given a force id it finds the associated limits.
         * If the specified force is not found it returns ForceLimits(0,0).
         * @param id Id of the force to find.
         * @return The limits of the specified force, ForceLimits(0,0) if not found. */
        static ForceLimits get_limits_from_id(unsigned int id)
        {
          std::map<unsigned int,ForceLimits>::const_iterator iter = id_2_limits.find(id);
          if(iter==id_2_limits.end())
            return ForceLimits();
          return iter->second;
        }

        static const std::map<std::string,unsigned int> name_2_id;
        static const std::map<unsigned int,std::string> id_2_name;
        static const std::map<unsigned int,ForceLimits> id_2_limits;
      };
      const std::map<std::string,unsigned int> ForceUtil::name_2_id = ForceUtil::create_name_2_id_map();
      const std::map<unsigned int,std::string> ForceUtil::id_2_name = ForceUtil::create_id_2_name_map(ForceUtil::name_2_id);
      const std::map<unsigned int,ForceLimits> ForceUtil::id_2_limits = ForceUtil::create_id_2_limits_map();


    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_hrp2_common_H__
