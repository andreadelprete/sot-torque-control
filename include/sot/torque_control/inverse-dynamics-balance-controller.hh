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
        DYNAMIC_GRAPH_ENTITY_DECL();
        
      public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /* --- CONSTRUCTOR ---- */
        InverseDynamicsBalanceController( const std::string & name );

        void init(const double& dt);

        /* --- SIGNALS --- */
        DECLARE_SIGNAL_IN(com_pos,                  ml::Vector);
        DECLARE_SIGNAL_IN(com_vel,                  ml::Vector);
        DECLARE_SIGNAL_IN(com_acc,                  ml::Vector);
        DECLARE_SIGNAL_IN(posture_pos,              ml::Vector);
        DECLARE_SIGNAL_IN(posture_vel,              ml::Vector);
        DECLARE_SIGNAL_IN(kp_base_orientation,      ml::Vector);
        DECLARE_SIGNAL_IN(kp_constraints,           ml::Vector);
        DECLARE_SIGNAL_IN(kd_constraints,           ml::Vector);
        DECLARE_SIGNAL_IN(w_com,                    ml::Vector);
        DECLARE_SIGNAL_IN(w_posture,                ml::Vector);
        DECLARE_SIGNAL_IN(w_base_orientation,       ml::Vector);
        DECLARE_SIGNAL_IN(w_torques,                ml::Vector);
        DECLARE_SIGNAL_IN(w_forces,                 ml::Vector);
        DECLARE_SIGNAL_IN(weight_contact_forces,    ml::Vector);
        DECLARE_SIGNAL_IN(mu,                       ml::Vector);
        DECLARE_SIGNAL_IN(contact_points,           ml::Vector);
        DECLARE_SIGNAL_IN(contact_normals,          ml::Vector);
        DECLARE_SIGNAL_IN(f_min,                    ml::Vector);
        DECLARE_SIGNAL_IN(tau_max,                  ml::Vector);
        DECLARE_SIGNAL_IN(q_min,                    ml::Vector);
        DECLARE_SIGNAL_IN(q_max,                    ml::Vector);
        DECLARE_SIGNAL_IN(dq_max,                   ml::Vector);
        DECLARE_SIGNAL_IN(ddq_max,                  ml::Vector);
        DECLARE_SIGNAL_IN(dt_joint_pos_limits,      double    );
        DECLARE_SIGNAL_IN(tau_estimated,            ml::Vector);
        DECLARE_SIGNAL_IN(q,                        ml::Vector);
        DECLARE_SIGNAL_IN(v,                        ml::Vector);
        DECLARE_SIGNAL_IN(base_contact_force,       ml::Vector);
        DECLARE_SIGNAL_IN(feet_forces,              ml::Vector);
        DECLARE_SIGNAL_IN(active_joints,            ml::Vector);
        
        DECLARE_SIGNAL_OUT(tau_des,                 ml::Vector);
        DECLARE_SIGNAL_OUT(f_des,                   ml::Vector);
        DECLARE_SIGNAL_OUT(com,                     ml::Vector);
        DECLARE_SIGNAL_OUT(base_orientation,        ml::Vector);
        DECLARE_SIGNAL_OUT(feet_pos_ref,            ml::Vector);
        DECLARE_SIGNAL_OUT(feet_pos,                ml::Vector);
        DECLARE_SIGNAL_OUT(dv_des,                  ml::Vector);
        

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
        bool              m_initSucceeded;    /// true if the entity has been successfully initialized
        double            m_dt;               /// control loop time period

        /// Integral of the joint tracking errors

      }; // class InverseDynamicsBalanceController
      
    }    // namespace torque_control
  }      // namespace sot
}        // namespace dynamicgraph



#endif // #ifndef __sot_torque_control_inverse_dynamics_balance_controller_H__
