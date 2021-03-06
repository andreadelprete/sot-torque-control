/*
 * Copyright 2017, Thomas Flayols, LAAS-CNRS
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

#include <sot/torque_control/free-flyer-locator.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/stop-watch.hh>

#include "pinocchio/algorithm/frames.hpp"

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
      using namespace se3;

      typedef Eigen::Vector6d Vector6;

#define PROFILE_FREE_FLYER_COMPUTATION          "Free-flyer position computation"
#define PROFILE_FREE_FLYER_VELOCITY_COMPUTATION "Free-flyer velocity computation"

#define INPUT_SIGNALS     m_base6d_encodersSIN << m_joint_velocitiesSIN
#define OUTPUT_SIGNALS    m_base6dFromFoot_encodersSOUT << m_freeflyer_aaSOUT << m_vSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef FreeFlyerLocator EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FreeFlyerLocator,
                                         "FreeFlyerLocator");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      FreeFlyerLocator::
          FreeFlyerLocator(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN( base6d_encoders,            ml::Vector)
            ,CONSTRUCT_SIGNAL_IN( joint_velocities,           ml::Vector)
            ,CONSTRUCT_SIGNAL_INNER(kinematics_computations,  ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(base6dFromFoot_encoders,    ml::Vector, m_kinematics_computationsSINNER)
            ,CONSTRUCT_SIGNAL_OUT(freeflyer_aa,               ml::Vector, m_base6dFromFoot_encodersSOUT)
            ,CONSTRUCT_SIGNAL_OUT(v,                          ml::Vector, m_kinematics_computationsSINNER)
            ,m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid1(*this, &FreeFlyerLocator::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "URDF file path (string)")));
      }

      void FreeFlyerLocator::init(const std::string& urdfFile)
      {
        try 
        {
          se3::urdf::buildModel(urdfFile,se3::JointModelFreeFlyer(),m_model);
          assert(m_model.nq == N_JOINTS+7);
          assert(m_model.nv == N_JOINTS+6);
          assert(m_model.existFrame(LEFT_FOOT_FRAME_NAME));
          assert(m_model.existFrame(RIGHT_FOOT_FRAME_NAME));
          m_left_foot_id = m_model.getFrameId(LEFT_FOOT_FRAME_NAME);
          m_right_foot_id = m_model.getFrameId(RIGHT_FOOT_FRAME_NAME);
          m_q_pin.setZero(m_model.nq);
          m_q_pin[6]= 1.; // for quaternion
          m_q_sot.setZero(N_JOINTS+6);
          m_v_pin.setZero(N_JOINTS+6);
          m_v_sot.setZero(N_JOINTS+6);
        } 
        catch (const std::exception& e) 
        { 
          std::cout << e.what();
          return SEND_MSG("Init failed: Could load URDF :" + urdfFile, MSG_TYPE_ERROR);
        }
        m_data = new se3::Data(m_model);
        m_initSucceeded = true;
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_INNER_FUNCTION(kinematics_computations, ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal kinematics_computations before initialization!");
          return s;
        }

        EIGEN_CONST_VECTOR_FROM_SIGNAL(q, m_base6d_encodersSIN(iter));     //n+6
        assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq, m_joint_velocitiesSIN(iter));
        assert(dq.size()==N_JOINTS     && "Unexpected size of signal joint_velocities");

        /* convert sot to pinocchio joint order */
        joints_sot_to_urdf(q.tail<N_JOINTS>(), m_q_pin.tail<N_JOINTS>());
        joints_sot_to_urdf(dq, m_v_pin.tail<N_JOINTS>());

        /* Compute kinematic and return q with freeflyer */
        se3::forwardKinematics(m_model, *m_data, m_q_pin, m_v_pin);
        se3::framesForwardKinematics(m_model, *m_data);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(base6dFromFoot_encoders,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal base6dFromFoot_encoders before initialization!");
          return s;
        }
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);
        
        m_kinematics_computationsSINNER(iter);

        getProfiler().start(PROFILE_FREE_FLYER_COMPUTATION);
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(q, m_base6d_encodersSIN(iter));     //n+6
          assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
                    
          /* Compute kinematic and return q with freeflyer */
          const se3::SE3 iMo1(m_data->oMf[m_left_foot_id].inverse());
          const se3::SE3 iMo2(m_data->oMf[m_right_foot_id].inverse());
          // Average in SE3
          const se3::SE3::Vector3 w(0.5*(se3::log3(iMo1.rotation())+se3::log3(iMo2.rotation())));
          m_Mff = se3::SE3(se3::exp3(w), 0.5 * (iMo1.translation()+iMo2.translation() ));

          // due to distance from ankle to ground
          Eigen::Map<const Eigen::Vector3d> righ_foot_sole_xyz(&RIGHT_FOOT_SOLE_XYZ[0]);

          m_q_sot.tail<N_JOINTS>() = q.tail<N_JOINTS>();
          base_se3_to_sot(m_Mff.translation()-righ_foot_sole_xyz,
                          m_Mff.rotation(),
                          m_q_sot.head<6>());

          EIGEN_VECTOR_TO_VECTOR(m_q_sot, s);
        }
        getProfiler().stop(PROFILE_FREE_FLYER_COMPUTATION);

        return s;
      }
      
      DEFINE_SIGNAL_OUT_FUNCTION(freeflyer_aa,ml::Vector)
      {
        m_base6dFromFoot_encodersSOUT(iter);
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal freeflyer_aa before initialization!");
          return s;
        }
        //oMi is has been calulated before since we depend on base6dFromFoot_encoders signal.
        //just read the data, convert to axis angle
        if(s.size()!=6)
          s.resize(6);
        //~ const se3::SE3 & iMo = m_data->oMi[31].inverse(); 
        const Eigen::AngleAxisd aa(m_Mff.rotation()); 
        Eigen::Vector6d freeflyer;
        freeflyer << m_Mff.translation(), aa.axis() * aa.angle();

        // due to distance from ankle to ground
        Eigen::Map<const Eigen::Vector3d> righ_foot_sole_xyz(&RIGHT_FOOT_SOLE_XYZ[0]);
        freeflyer.head<3>() -= righ_foot_sole_xyz;

        EIGEN_VECTOR_TO_VECTOR(freeflyer,s);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(v,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal v before initialization!");
          return s;
        }
        if(s.size()!=N_JOINTS+6)
          s.resize(N_JOINTS+6);

        m_kinematics_computationsSINNER(iter);

        getProfiler().start(PROFILE_FREE_FLYER_VELOCITY_COMPUTATION);
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(dq, m_joint_velocitiesSIN(iter));
          assert(dq.size()==N_JOINTS     && "Unexpected size of signal joint_velocities");

          /* Compute foot velocities */
          const Frame & f_lf = m_model.frames[m_left_foot_id];
          const Motion v_lf_local = f_lf.placement.actInv(m_data->v[f_lf.parent]);
          const Vector6 v_lf = m_data->oMf[m_left_foot_id].act(v_lf_local).toVector();

          const Frame & f_rf = m_model.frames[m_right_foot_id];
          const Motion v_rf_local = f_rf.placement.actInv(m_data->v[f_rf.parent]);
          const Vector6 v_rf = m_data->oMf[m_right_foot_id].act(v_rf_local).toVector();

          m_v_sot.head<6>() = - 0.5*(v_lf + v_rf);
          m_v_sot.tail<N_JOINTS>() = dq;

          EIGEN_VECTOR_TO_VECTOR(m_v_sot, s);
        }
        getProfiler().stop(PROFILE_FREE_FLYER_VELOCITY_COMPUTATION);

        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void FreeFlyerLocator::display(std::ostream& os) const
      {
        os << "FreeFlyerLocator "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

      void FreeFlyerLocator::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "FreeFlyerLocator:\n"
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

