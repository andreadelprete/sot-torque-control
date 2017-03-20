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

#define INPUT_SIGNALS         m_com_ref_posSIN \
                           << m_com_ref_velSIN \
                           << m_com_ref_accSIN \
                           << m_posture_ref_posSIN \
                           << m_posture_ref_velSIN \
                           << m_posture_ref_accSIN \
                           << m_base_orientation_ref_posSIN \
                           << m_base_orientation_ref_velSIN \
                           << m_base_orientation_ref_accSIN \
                           << m_kp_base_orientationSIN \
                           << m_kd_base_orientationSIN \
                           << m_kp_constraintsSIN \
                           << m_kd_constraintsSIN \
                           << m_kp_comSIN \
                           << m_kd_comSIN \
                           << m_kp_postureSIN \
                           << m_kd_postureSIN \
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
                           << m_wrench_left_footSIN  \
                           << m_wrench_right_footSIN  \
                           << m_active_jointsSIN

#define OUTPUT_SIGNALS        m_tau_desSOUT \
                           << m_f_desSOUT \
                           << m_comSOUT \
                           << m_base_orientationSOUT \
                           << m_right_foot_posSOUT \
                           << m_left_foot_posSOUT \
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
            ,CONSTRUCT_SIGNAL_IN(com_ref_pos,                 ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_ref_vel,                 ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(com_ref_acc,                 ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_pos,             ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_vel,             ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(posture_ref_acc,             ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_pos,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_vel,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(base_orientation_ref_acc,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_base_orientation,         ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_base_orientation,         ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_constraints,              ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_constraints,              ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_com,                      ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_com,                      ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kp_posture,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(kd_posture,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_com,                    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_posture,                ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_base_orientation,       ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_torques,                ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(w_forces,                 ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(weight_contact_forces,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(mu,                       double)
            ,CONSTRUCT_SIGNAL_IN(contact_points,           ml::Matrix)
            ,CONSTRUCT_SIGNAL_IN(contact_normals,          ml::Matrix)
            ,CONSTRUCT_SIGNAL_IN(f_min,                    double)
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
            ,CONSTRUCT_SIGNAL_IN(wrench_left_foot,         ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(wrench_right_foot,        ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(active_joints,            ml::Vector)
            ,CONSTRUCT_SIGNAL_OUT(tau_des,                 ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(f_des,                   ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(dv_des,                  ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(com,                     ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(base_orientation,        ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(left_foot_pos,           ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(right_foot_pos,          ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_INNER(active_joints_checked, ml::Vector, m_active_jointsSIN)
            ,m_initSucceeded(false)
            ,m_enabled(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid2(*this, &InverseDynamicsBalanceController::init,
                                    docCommandVoid2("Initialize the entity.",
                                                    "Time period in seconds (double)",
                                                    "URDF file path (string)")));
      }

      void InverseDynamicsBalanceController::init(const double& dt, const std::string& urdfFile)
      {
        if(dt<=0.0)
          return SEND_MSG("Init failed: Timestep must be positive", MSG_TYPE_ERROR);
        //~ if(!m_qSIN.isPlugged())
          //~ return SEND_MSG("Init failed: Signal q is not plugged", MSG_TYPE_ERROR);
        try 
        {
          se3::urdf::buildModel(urdfFile,se3::JointModelFreeFlyer(),m_model);
        } 
        catch (const std::exception& e) 
        { 
          std::cout << e.what();
          return SEND_MSG("Init failed: Could load URDF :" + urdfFile, MSG_TYPE_ERROR);
        }
        m_data = new se3::Data(m_model);
        cout<<m_model;  
        m_dt = dt;
        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      /** Copy active_joints only if a valid transition occurs. (From all OFF) or (To all OFF)**/
      DEFINE_SIGNAL_INNER_FUNCTION(active_joints_checked, ml::Vector)
      {
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);

        EIGEN_CONST_VECTOR_FROM_SIGNAL(active_joints, m_active_jointsSIN(iter));
        if (m_enabled == false)
        {
          if (active_joints.any())
          {
              /* from all OFF to some ON */
              m_enabled = true ;
              EIGEN_VECTOR_TO_VECTOR(active_joints,s);
          }
        }
        else if (!active_joints.any())
        {
            /* from some ON to all OFF */
            m_enabled = false ;
        }
        if (m_enabled == false)
          for(int i=0; i<N_JOINTS; i++)
            s(i)=false;
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(tau_des,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal tau_des before initialization!");
          return s;
        }
        getProfiler().start(PROFILE_TAU_DES_COMPUTATION);
        //EIGEN_VECTOR_TO_VECTOR(...,s);
        getProfiler().stop(PROFILE_TAU_DES_COMPUTATION);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(dv_des,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal dv_des before initialization!");
          return s;
        }
        /*
         * Code
         */
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(f_des,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal f_des before initialization!");
          return s;
        }
        /*
         * Code
         */
        return s;
      }
      
      
      DEFINE_SIGNAL_OUT_FUNCTION(com,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal com before initialization!");
          return s;
        }
        /*
         * Code
         */
        return s;
      }
      
      
      DEFINE_SIGNAL_OUT_FUNCTION(base_orientation,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal base_orientation before initialization!");
          return s;
        }
        /*
         * Code
         */
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(left_foot_pos, ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal left_foot_pos before initialization!");
          return s;
        }
        /*
         * Code
         */
        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(right_foot_pos, ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal right_foot_pos before initialization!");
          return s;
        }
        /*
         * Code
         */
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

