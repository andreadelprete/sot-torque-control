/*
 * Copyright 2017, Andrea Del Prete, LAAS-CNRS
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

#ifndef __sot_torque_control_inverse_dynamics_balance_controller_H__
#define __sot_torque_control_inverse_dynamics_balance_controller_H__

/* --------------------------------------------------------------------- */
/* --- API ------------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#if defined (WIN32)
#  if defined (inverse_dynamics_balance_controller_EXPORTS)
#    define SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT __declspec(dllexport)
#  else
#    define SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT __declspec(dllimport)
#  endif
#else
#  define SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT
#endif


/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <sot/torque_control/signal-helper.hh>
#include <sot/torque_control/utils/vector-conversions.hh>
#include <sot/torque_control/utils/logger.hh>
#include <sot/torque_control/hrp2-common.hh>
#include <map>
#include <initializer_list>
#include "boost/assign.hpp"

/* Pinocchio */
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <pininvdyn/robot-wrapper.hpp>
#include <pininvdyn/solvers/solver-HQP-base.hpp>
#include <pininvdyn/contacts/contact-6d.hpp>
#include <pininvdyn/inverse-dynamics-formulation-acc-force.hpp>
#include <pininvdyn/tasks/task-com-equality.hpp>
#include <pininvdyn/tasks/task-joint-posture.hpp>
#include <pininvdyn/trajectories/trajectory-euclidian.hpp>

namespace dynamicgraph {
  namespace sot {
    namespace torque_control {

      /* --------------------------------------------------------------------- */
      /* --- CLASS ----------------------------------------------------------- */
      /* --------------------------------------------------------------------- */

      class SOTINVERSEDYNAMICSBALANCECONTROLLER_EXPORT InverseDynamicsBalanceController
	:public::dynamicgraph::Entity
      {
        typedef InverseDynamicsBalanceController EntityClassName;
	typedef Eigen::Matrix<double,N_JOINTS,1> VectorN;
	typedef Eigen::Matrix<double,3,1>        Vector3;
	typedef pininvdyn::math::Vector6         Vector6;
        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        InverseDynamicsBalanceController( const std::string & name );

        void init(const double& dt, const std::string& urdfFile);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(com_ref_pos,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(com_ref_vel,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(com_ref_acc,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(posture_ref_pos,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(posture_ref_vel,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(posture_ref_acc,            dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(base_orientation_ref_pos,   dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(base_orientation_ref_vel,   dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(base_orientation_ref_acc,   dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(kp_base_orientation,        dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_base_orientation,        dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_constraints,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_constraints,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_com,                     dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_com,                     dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_posture,                 dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_posture,                 dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kp_pos,                     dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(kd_pos,                     dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(w_com,                      double);
        DECLARE_SIGNAL_IN(w_posture,                  double);
        DECLARE_SIGNAL_IN(w_base_orientation,         double);
        DECLARE_SIGNAL_IN(w_torques,                  double);
        DECLARE_SIGNAL_IN(w_forces,                   double);
        DECLARE_SIGNAL_IN(weight_contact_forces,      dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(mu,                         double);
        DECLARE_SIGNAL_IN(contact_points,             dynamicgraph::Matrix);
        DECLARE_SIGNAL_IN(contact_normal,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(f_min,                      double);

        DECLARE_SIGNAL_IN(rotor_inertias,             dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(gear_ratios,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(tau_max,                    dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(q_min,                      dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(q_max,                      dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(dq_max,                     dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(ddq_max,                    dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(dt_joint_pos_limits,        double);

        DECLARE_SIGNAL_IN(tau_estimated,              dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(q,                          dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(v,                          dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(wrench_base,                dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(wrench_left_foot,           dynamicgraph::Vector);
        DECLARE_SIGNAL_IN(wrench_right_foot,          dynamicgraph::Vector);

        DECLARE_SIGNAL_IN(active_joints,              dynamicgraph::Vector); /// mask with 1 for controlled joints, 0 otherwise
        
        DECLARE_SIGNAL_OUT(tau_des,                   dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(dv_des,                    dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(f_des_right_foot,          Vector6);
        DECLARE_SIGNAL_OUT(f_des_left_foot,           Vector6);
        DECLARE_SIGNAL_OUT(zmp_des_right_foot,        Eigen::Vector3d);
        DECLARE_SIGNAL_OUT(zmp_des_left_foot,         Eigen::Vector3d);
        DECLARE_SIGNAL_OUT(zmp_des_right_foot_local,  Eigen::Vector3d);
        DECLARE_SIGNAL_OUT(zmp_des_left_foot_local,   Eigen::Vector3d);
        DECLARE_SIGNAL_OUT(zmp_des,                   Eigen::Vector3d);
        DECLARE_SIGNAL_OUT(com,                       Vector3);
        DECLARE_SIGNAL_OUT(base_orientation,          dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(right_foot_pos,            dynamicgraph::Vector);
        DECLARE_SIGNAL_OUT(left_foot_pos,             dynamicgraph::Vector);
        
        /// This signal copies active_joints only if it changes from a all false or to an all false value
        DECLARE_SIGNAL_INNER(active_joints_checked, dynamicgraph::Vector);
        
        /* --- COMMANDS --- */
        /* --- ENTITY INHERITANCE --- */
        virtual void display( std::ostream& os ) const;
        virtual void commandLine(const std::string& cmdLine,
                                 std::istringstream& cmdArgs,
                                 std::ostream& os);

        void sendMsg(const std::string& msg, MsgType t=MSG_TYPE_INFO, const char* file="", int line=0)
        {
          getLogger().sendMsg("[InverseDynamicsBalanceController-"+name+"] "+msg, t, file, line);
        }
        
      protected:
        double            m_dt;               /// control loop time period
        double            m_t;
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        bool              m_enabled;          /// True if controler is enabled
        bool              m_firstTime;        /// True at the first iteration of the controller

        /// pininvdyn
        pininvdyn::RobotWrapper *                       m_robot;
        pininvdyn::solvers::Solver_HQP_base *           m_hqpSolver;
        pininvdyn::InverseDynamicsFormulationAccForce * m_invDyn;
        pininvdyn::contacts::Contact6d *                m_contactRF;
        pininvdyn::contacts::Contact6d *                m_contactLF;
        pininvdyn::tasks::TaskComEquality *             m_taskCom;
        pininvdyn::tasks::TaskJointPosture *            m_taskPosture;
        pininvdyn::tasks::TaskJointPosture *            m_taskBlockedJoints;

        pininvdyn::trajectories::TrajectorySample       m_sampleCom;
        pininvdyn::trajectories::TrajectorySample       m_samplePosture;

        pininvdyn::math::Vector  m_dv_sot;
        pininvdyn::math::Vector  m_f;         /// force coefficients (24d)
        pininvdyn::math::Vector6 m_f_RF;      /// 6d wrench right foot
        pininvdyn::math::Vector6 m_f_LF;      /// 6d wrench left foot
        pininvdyn::math::Vector3 m_zmp_LF;    /// 3d zmp left foot
        pininvdyn::math::Vector3 m_zmp_RF;    /// 3d zmp left foot
        pininvdyn::math::Vector3 m_zmp;       /// 3d global zmp
        pininvdyn::math::Vector  m_tau_sot;
        pininvdyn::math::Vector  m_q_urdf;
        pininvdyn::math::Vector  m_v_urdf;

        unsigned int m_timeLast;
        
      }; // class InverseDynamicsBalanceController
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_inverse_dynamics_balance_controller_H__
