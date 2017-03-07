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

#include <sot/torque_control/inverse-dynamics-balance-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
//Size to be aligned                "-------------------------------------------------------"
#define PROFILE_TAU_DES_COMPUTATION "InverseDynamicsBalanceController: desired tau          "

#define INPUT_SIGNALS         m_com_posSIN \
                           << m_com_velSIN \
                           << m_com_accSIN \
                           << m_posture_posSIN \
                           << m_posture_velSIN \
                           << m_kp_base_orientationSIN \
                           << m_kp_constraintsSIN \
                           << m_kd_constraintsSIN \
                           << m_w_comSIN \
                           << m_w_postureSIN \
                           << m_w_base_orientationSIN \
                           << m_w_torquesSIN \
                           << m_w_forcesSIN \
                           << m_weight_contact_forcesSIN \
                           << m_muSIN \
                           << m_contact_pointsSIN \
                           << m_contact_normalsSIN \
                           << m_f_minSIN \
                           << m_tau_maxSIN \
                           << m_q_minSIN \
                           << m_q_maxSIN \
                           << m_dq_maxSIN \
                           << m_ddq_maxSIN \
                           << m_dt_joint_pos_limitsSIN \
                           << m_tau_estimatedSIN \
                           << m_qSIN \
                           << m_vSIN \
                           << m_base_contact_forceSIN \
                           << m_feet_forcesSIN  \
                           << m_active_jointsSIN

#define OUTPUT_SIGNALS        m_tau_desSOUT \
                           << m_f_desSOUT \
                           << m_comSOUT \
                           << m_base_orientationSOUT \
                           << m_feet_pos_refSOUT \
                           << m_feet_posSOUT \
                           << m_dv_desSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef InverseDynamicsBalanceController EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InverseDynamicsBalanceController,
                                         "InverseDynamicsBalanceController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      InverseDynamicsBalanceController::
          InverseDynamicsBalanceController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(com_pos,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_vel,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_acc,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_pos,              ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_vel,              ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_base_orientation,      ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_constraints,           ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_constraints,           ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_com,                    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_posture,                ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_base_orientation,       ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_torques,                ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_forces,                 ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(weight_contact_forces,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(mu,                       ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(contact_points,           ml::Vector)// a matrix?
            ,CONSTRUCT_SIGNAL_IN(contact_normals,          ml::Vector)// a matrix?
            ,CONSTRUCT_SIGNAL_IN(f_min,                    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(tau_max,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(q_min,                    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(q_max,                    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(dq_max,                   ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(ddq_max,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(dt_joint_pos_limits,      double    )
            ,CONSTRUCT_SIGNAL_IN(tau_estimated,            ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(q,                        ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(v,                        ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_contact_force,       ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(feet_forces,              ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(active_joints,            ml::Vector)
            ,CONSTRUCT_SIGNAL_OUT(tau_des,                 ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(f_des,                   ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(com,                     ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(base_orientation,        ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(feet_pos_ref,            ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(feet_pos,                ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(dv_des,                  ml::Vector, INPUT_SIGNALS)
            
            ,m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid1(*this, &InverseDynamicsBalanceController::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Time period in seconds (double)")));
      }

      void InverseDynamicsBalanceController::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_qSIN.isPlugged())
          return SEND_MSG("Init failed: signal q is not plugged", MSG_TYPE_ERROR);
          //TODO add other tests on mandatory signals
        m_dt = dt;
        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(tau_des,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal pwmDes before initialization!");
          return s;
        }
        getProfiler().start(PROFILE_TAU_DES_COMPUTATION);
        //EIGEN_VECTOR_TO_VECTOR(...,s);
        getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);
        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void InverseDynamicsBalanceController::display(std::ostream& os) const
      {
        os << "InverseDynamicsBalanceController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

      void InverseDynamicsBalanceController::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotInverseDynamicsBalanceController:\n"
              << "\t -." << std::endl;
          Entity::commandLine(cmdLine, cmdArgs, os);
        }
        else
        {
          Entity::commandLine(cmdLine,cmdArgs,os);
        }
      }
      
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

