/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
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

#include <sot/torque_control/position-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <sot/torque_control/utils/metapod-helper.hh>
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
      using namespace metapod;
//Size to be aligned                "-------------------------------------------------------"
#define PROFILE_PWM_DES_COMPUTATION "PositionController: desired pwm computation            "

#define GAIN_SIGNALS      m_KpSIN << m_KdSIN << m_KiSIN
#define REF_JOINT_SIGNALS m_qRefSIN << m_dqRefSIN
#define STATE_SIGNALS     m_base6d_encodersSIN << m_jointsVelocitiesSIN

#define INPUT_SIGNALS     STATE_SIGNALS << REF_JOINT_SIGNALS << GAIN_SIGNALS

#define OUTPUT_SIGNALS m_pwmDesSOUT << m_qErrorSOUT

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef PositionController EntityClassName;

      /* --- DG FACTORY ---------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PositionController,
                                         "PositionController");

      /* ------------------------------------------------------------------- */
      /* --- CONSTRUCTION -------------------------------------------------- */
      /* ------------------------------------------------------------------- */
      PositionController::
          PositionController(const std::string& name)
            : Entity(name)
            ,CONSTRUCT_SIGNAL_IN(base6d_encoders,     ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(jointsVelocities,    ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(qRef,                ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(dqRef,               ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kp,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(Kd,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_IN(Ki,                  ml::Vector)
            ,CONSTRUCT_SIGNAL_OUT(pwmDes,             ml::Vector, INPUT_SIGNALS)
            ,CONSTRUCT_SIGNAL_OUT(qError,             ml::Vector, m_base6d_encodersSIN <<
                                                                  m_qRefSIN)
            ,m_initSucceeded(false)
      {
        Entity::signalRegistration( INPUT_SIGNALS << OUTPUT_SIGNALS );

        /* Commands. */
        addCommand("init",
                   makeCommandVoid1(*this, &PositionController::init,
                                    docCommandVoid1("Initialize the entity.",
                                                    "Time period in seconds (double)")));
        addCommand("resetIntegral",
                   makeCommandVoid0(*this, &PositionController::resetIntegral,
                                    docCommandVoid0("Reset the integral.")));
      }

      void PositionController::init(const double& dt)
      {
        if(dt<=0.0)
          return SEND_MSG("Timestep must be positive", MSG_TYPE_ERROR);
        if(!m_base6d_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal base6d_encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_qRefSIN.isPlugged())
          return SEND_MSG("Init failed: signal qRef is not plugged", MSG_TYPE_ERROR);
        if(!m_dqRefSIN.isPlugged())
          return SEND_MSG("Init failed: signal dqRef is not plugged", MSG_TYPE_ERROR);
        if(!m_KpSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kp is not plugged", MSG_TYPE_ERROR);
        if(!m_KdSIN.isPlugged())
          return SEND_MSG("Init failed: signal Kd is not plugged", MSG_TYPE_ERROR);
        if(!m_KiSIN.isPlugged())
          return SEND_MSG("Init failed: signal Ki is not plugged", MSG_TYPE_ERROR);

        m_dt = dt;
        m_pwmDes.setZero(N_JOINTS);
        m_q.setZero();
        m_dq.setZero();

        resetIntegral();

        m_initSucceeded = true;
      }

      void PositionController::resetIntegral()
      {
        m_e_integral.setZero(N_JOINTS);
      }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(pwmDes,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal pwmDes before initialization!");
          return s;
        }

        getProfiler().start(PROFILE_PWM_DES_COMPUTATION);
        {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(Kp,          m_KpSIN(iter)); // n
          EIGEN_CONST_VECTOR_FROM_SIGNAL(Kd,          m_KdSIN(iter)); // n
          EIGEN_CONST_VECTOR_FROM_SIGNAL(q,           m_base6d_encodersSIN(iter));     //n+6
          EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,          m_jointsVelocitiesSIN(iter));     // n
          EIGEN_CONST_VECTOR_FROM_SIGNAL(qRef,        m_qRefSIN(iter));   // n
          EIGEN_CONST_VECTOR_FROM_SIGNAL(dqRef,       m_dqRefSIN(iter));  // n

          assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
          assert(dq.size()==N_JOINTS      && "Unexpected size of signal dq");
          assert(qRef.size()==N_JOINTS    && "Unexpected size of signal qRef");
          assert(dqRef.size()==N_JOINTS   && "Unexpected size of signal dqRef");
          assert(Kp.size()==N_JOINTS      && "Unexpected size of signal Kd");
          assert(Kd.size()==N_JOINTS      && "Unexpected size of signal Kd");

          m_pwmDes = Kp.cwiseProduct(qRef-q.tail<N_JOINTS>()) + Kd.cwiseProduct(dqRef-dq);

          EIGEN_VECTOR_TO_VECTOR(m_pwmDes,s);
        }
        getProfiler().stop(PROFILE_PWM_DES_COMPUTATION);

        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(qError,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_MSG("Cannot compute signal qError before initialization!",MSG_TYPE_WARNING_STREAM);
          return s;
        }

        EIGEN_CONST_VECTOR_FROM_SIGNAL(q,           m_base6d_encodersSIN(iter));     //n+6
        EIGEN_CONST_VECTOR_FROM_SIGNAL(qRef,        m_qRefSIN(iter));   // n
        assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
        assert(qRef.size()==N_JOINTS    && "Unexpected size of signal qRef");

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(unsigned int i=0; i<N_JOINTS; i++)
          s(i)= qRef[i]-q(6+i);

        return s;
      }

      /* --- COMMANDS ---------------------------------------------------------- */

      /* ------------------------------------------------------------------- */
      /* --- ENTITY -------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      void PositionController::display(std::ostream& os) const
      {
        os << "PositionController "<<getName();
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

      void PositionController::commandLine(const std::string& cmdLine,
                                            std::istringstream& cmdArgs,
                                            std::ostream& os )
      {
        if( cmdLine == "help" )
        {
          os << "sotPositionController:\n"
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

