/*
 * Copyright 2015, Andrea Del Prete, LAAS-CNRS
 *
 * This file is part of sot-torque-control.
 * sot-dyninv is free software: you can redistribute it and/or
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

#include <sot/torque_control/joint-torque-controller.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>

#include <sot/torque_control/commands-helper.hh>
#include <Eigen/Dense>

namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {

#define MODEL_INPUT_SIGNALS     m_k_tauSIN << m_k_vSIN \
                             << m_motorParameterKt_pSIN << m_motorParameterKt_nSIN \
                             << m_motorParameterKf_pSIN << m_motorParameterKf_nSIN \
                             << m_motorParameterKv_pSIN << m_motorParameterKv_nSIN \
                             << m_motorParameterKa_pSIN << m_motorParameterKa_nSIN <<  m_polySignDqSIN
//                              <<m_f_k1pSIN << m_f_k2pSIN << m_f_k3pSIN << m_f_k1nSIN << m_f_k2nSIN << m_f_k3nSIN << \
//                                m_f_q1pSIN << m_f_q2pSIN << m_f_q3pSIN << m_f_q1nSIN << m_f_q2nSIN << m_f_q3nSIN << \
//                                m_f_tau1pSIN << m_f_tau2pSIN << m_f_tau1nSIN << m_f_tau2nSIN << \
//                                m_g_k1pSIN << m_g_k2pSIN << m_g_k3pSIN << m_g_k1nSIN << m_g_k2nSIN << m_g_k3nSIN << \
//                                m_g_q1pSIN << m_g_q2pSIN << m_g_q3pSIN << m_g_q1nSIN << m_g_q2nSIN << m_g_q3nSIN << \
//                                m_g_dq1pSIN << m_g_dq2pSIN << m_g_dq1nSIN << m_g_dq2nSIN << \
//                                m_dq_thresholdSIN << m_ddq_thresholdSIN

#define ESTIMATOR_INPUT_SIGNALS m_base6d_encodersSIN << m_jointsVelocitiesSIN << m_jointsAccelerationsSIN << \
                                m_jointsTorquesSIN

#define TORQUE_CONTROL_INPUT_SIGNALS    m_jointsTorquesDesiredSIN << m_KpTorqueSIN  << m_KiTorqueSIN  << m_frictionCompensationPercentageSIN//<< m_activeJointsSIN
#define CURRENT_CONTROL_INPUT_SIGNALS   m_measuredCurrentSIN      << m_KpCurrentSIN << m_KiCurrentSIN 
#define ALL_INPUT_SIGNALS       m_pwmSIN << m_tauFFSIN << m_tauFBSIN << \
                                ESTIMATOR_INPUT_SIGNALS << TORQUE_CONTROL_INPUT_SIGNALS << CURRENT_CONTROL_INPUT_SIGNALS << MODEL_INPUT_SIGNALS

#define ALL_OUTPUT_SIGNALS      m_desiredCurrentSOUT << m_controlCurrentSOUT << m_predictedJointsTorquesSOUT << \
                                m_predictedPwmSOUT << m_predictedPwm_tauSOUT << \
                                m_pwm_ffSOUT << m_pwm_fbSOUT << m_pwm_frictionSOUT << m_smoothSignDqSOUT

#define N_JOINTS 30

      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;
      using namespace std;
      using namespace metapod;
      using namespace Eigen;

      /// Define EntityClassName here rather than in the header file
      /// so that it can be used by the macros DEFINE_SIGNAL_**_FUNCTION.
      typedef JointTorqueController EntityClassName;

      /* --- DG FACTORY ------------------------------------------------------- */
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(JointTorqueController,"JointTorqueController");

      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      /* --- CONSTRUCTION ----------------------------------------------------- */
      JointTorqueController::
      JointTorqueController( const std::string & name )
        : Entity(name),
         CONSTRUCT_SIGNAL_IN(base6d_encoders,        ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(pwm,                    ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsVelocities,       ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsAccelerations,    ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsTorques,          ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(jointsTorquesDesired,   ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(measuredCurrent,        ml::Vector) 
        ,CONSTRUCT_SIGNAL_IN(KpTorque,                     ml::Vector)   // proportional gain for torque feedback controller
        ,CONSTRUCT_SIGNAL_IN(KiTorque,                     ml::Vector)   // integral gain for torque feedback controller
        ,CONSTRUCT_SIGNAL_IN(KpCurrent,                     ml::Vector)  // proportional gain for current feedback controller
        ,CONSTRUCT_SIGNAL_IN(KiCurrent,                     ml::Vector)  // integral gain for current feedback controller  
        ,CONSTRUCT_SIGNAL_IN(k_tau,                  ml::Vector)// to be del
        ,CONSTRUCT_SIGNAL_IN(k_v,                    ml::Vector)// to be del
        ,CONSTRUCT_SIGNAL_IN(frictionCompensationPercentage, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKt_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKf_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKv_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_p, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(motorParameterKa_n, ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(polySignDq        , ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(tauFF,                  ml::Vector)
        ,CONSTRUCT_SIGNAL_IN(tauFB,                  ml::Vector)
        ,CONSTRUCT_SIGNAL_OUT(desiredCurrent,        ml::Vector,   ESTIMATOR_INPUT_SIGNALS <<
                                                                   TORQUE_CONTROL_INPUT_SIGNALS <<
                                                                   MODEL_INPUT_SIGNALS )

        ,CONSTRUCT_SIGNAL_OUT(controlCurrent,        ml::Vector,   m_desiredCurrentSOUT <<
                                                                   CURRENT_CONTROL_INPUT_SIGNALS )
        ,CONSTRUCT_SIGNAL_OUT(predictedJointsTorques,  ml::Vector, m_pwmSIN<<
                                                                   m_jointsVelocitiesSIN<<
                                                                   m_k_tauSIN<<
                                                                   m_k_vSIN)
        ,CONSTRUCT_SIGNAL_OUT(predictedPwm,            ml::Vector, ESTIMATOR_INPUT_SIGNALS <<
                                                                   MODEL_INPUT_SIGNALS)
        ,CONSTRUCT_SIGNAL_OUT(predictedPwm_tau,        ml::Vector, ESTIMATOR_INPUT_SIGNALS <<
                                                                   MODEL_INPUT_SIGNALS)
        ,CONSTRUCT_SIGNAL_OUT(pwm_ff,              ml::Vector, m_tauFFSIN <<
                                                                  m_k_tauSIN)
        ,CONSTRUCT_SIGNAL_OUT(pwm_fb,              ml::Vector, m_tauFBSIN <<
                                                                  m_jointsTorquesSIN <<
                                                                  m_jointsTorquesDesiredSIN <<
                                                                  m_k_tauSIN <<
                                                                  m_KpTorqueSIN)
        ,CONSTRUCT_SIGNAL_OUT(pwm_friction,        ml::Vector, m_jointsVelocitiesSIN <<
                                                                  m_k_vSIN)
        ,CONSTRUCT_SIGNAL_OUT(smoothSignDq,        ml::Vector, m_jointsVelocitiesSIN )


      {
        Entity::signalRegistration( ALL_INPUT_SIGNALS << ALL_OUTPUT_SIGNALS);
        m_firstIter = true;

        /* Commands. */
        addCommand("getTimestep", makeDirectGetter(*this,&m_dt,
                                  docDirectGetter("Control timestep","double")));
        addCommand("getActiveJoints", makeDirectGetter(*this,&m_activeJointsString,
                                      docDirectGetter("Active joints","bool")));

        addCommand("init", makeCommandVoid1(*this, &JointTorqueController::init,
                              docCommandVoid1("Initialize the controller.",
                                              "Control timestep [s].")));
        addCommand("activate",
                   makeCommandVoid1(*this, &JointTorqueController::activate,
                                    docCommandVoid1("Activate torque control of the specified joint.",
                                                    "Joint name (string)")));
        addCommand("deactivate",
                   makeCommandVoid1(*this, &JointTorqueController::deactivate,
                                    docCommandVoid1("Deactivate torque control of the specified joint.",
                                                    "Joint name (string)")));
      }


      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      /* --- COMMANDS ---------------------------------------------------------- */
      void JointTorqueController::init(const double &timestep)
      {
        assert(timestep>0.0 && "Timestep should be > 0");
        if(!m_base6d_encodersSIN.isPlugged())
          return SEND_MSG("Init failed: signal base6d_encoders is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsVelocitiesSIN.isPlugged())
          return SEND_MSG("Init failed: signal jointsVelocities is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsTorquesSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_jointsTorquesSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_jointsTorquesDesiredSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_jointsTorquesDesiredSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_k_tauSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_k_tauSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_k_vSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_k_vSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KpTorqueSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KpTorqueSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KiTorqueSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KiTorqueSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KpCurrentSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KpCurrentSIN is not plugged", MSG_TYPE_ERROR);
        if(!m_KiCurrentSIN.isPlugged())
          return SEND_MSG("Init failed: signal m_KiCurrentSIN is not plugged", MSG_TYPE_ERROR);
        m_dt = timestep;
        m_firstIter = true;
        m_tau_star.setZero(N_JOINTS);
        m_current_star.setZero(N_JOINTS);
        m_f.setZero(N_JOINTS);
        m_g.setZero(N_JOINTS);
        m_current_des.setZero(N_JOINTS);
        m_tauErrIntegral.setZero(N_JOINTS);
        m_currentErrIntegral.setZero(N_JOINTS);
        m_qDes_for_position_controlled_joints.setZero(N_JOINTS);
        m_activeJoints.resize(N_JOINTS,true);
        updateActiveJointsString();
      }

      void JointTorqueController::activate(const string& jointName)
      {
        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;

        if(m_activeJoints[i]==false)
        {
          SEND_MSG("Activate joint "+jointName, MSG_TYPE_INFO);
          m_activeJoints[i] = true;
          updateActiveJointsString();
        }
        else
          SEND_MSG("Joint "+jointName+" is already active.", MSG_TYPE_WARNING);
      }

      void JointTorqueController::deactivate(const string& jointName)
      {
        unsigned int i;
        if(convertJointNameToJointId(jointName,i)==false)
          return;

        if(m_activeJoints[i]==true)
        {
          SEND_MSG("Deactivate joint "+jointName, MSG_TYPE_INFO);
          const ml::Vector& base6d_encoders = m_base6d_encodersSIN.accessCopy();
          m_qDes_for_position_controlled_joints[i] = base6d_encoders(6+i);
          m_activeJoints[i] = false;
          updateActiveJointsString();
        }
        else
          SEND_MSG("Joint "+jointName+" is already deactivated.", MSG_TYPE_WARNING);
      }

      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */
      /* --- SIGNALS ---------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(desiredCurrent, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(q,             m_base6d_encodersSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,            m_jointsVelocitiesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(ddq,           m_jointsAccelerationsSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau,           m_jointsTorquesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau_d,         m_jointsTorquesDesiredSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(kp,            m_KpTorqueSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(ki,            m_KiTorqueSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_tau,         m_k_tauSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_v,           m_k_vSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(frictionCompensationPercentage, m_frictionCompensationPercentageSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKt_p, m_motorParameterKt_pSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKt_n, m_motorParameterKt_nSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKf_p, m_motorParameterKf_pSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKf_n, m_motorParameterKf_nSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKv_p, m_motorParameterKv_pSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKv_n, m_motorParameterKv_nSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKa_p, m_motorParameterKa_pSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(motorParameterKa_n, m_motorParameterKa_nSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(polySignDq        , m_polySignDqSIN(iter));


//        EIGEN_CONST_VECTOR_FROM_SIGNAL(activeJoints,  m_activeJointsSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(ddq,           m_jointsAccelerationsSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq_thr,        m_dq_thresholdSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(ddq_thr,       m_ddq_thresholdSIN(iter));

        if(m_firstIter)
        {
          m_qDes_for_position_controlled_joints = q.tail<N_JOINTS>();
          m_firstIter = false;
        }

        m_tauErrIntegral += m_dt * ki.cwiseProduct(tau_d-tau);
        m_tau_star = tau_d + kp.cwiseProduct(tau_d - tau) + m_tauErrIntegral;
        if(dq.size()==N_JOINTS)
            for(int i=0; i<N_JOINTS; i++)
            {
                m_current_des(i) = motorModel.getCurrent(m_tau_star(i), dq(i), ddq(i),
                                                         motorParameterKt_p(i), motorParameterKt_n(i),
                                                         motorParameterKf_p(i)*frictionCompensationPercentage(i), motorParameterKf_n(i)*frictionCompensationPercentage(i),
                                                         motorParameterKv_p(i), motorParameterKv_n(i),
                                                         motorParameterKa_p(i), motorParameterKa_n(i) , polySignDq(i));
            }
        else if(dq.size()==N_JOINTS+6)
            for(int i=0; i<N_JOINTS; i++)
            {
                m_current_des(i) = motorModel.getCurrent(m_tau_star(i), dq(i+6), ddq(i+6),
                                                         motorParameterKt_p(i), motorParameterKt_n(i),
                                                         motorParameterKf_p(i)*frictionCompensationPercentage(i), motorParameterKf_n(i)*frictionCompensationPercentage(i),
                                                         motorParameterKv_p(i), motorParameterKv_n(i),
                                                         motorParameterKa_p(i), motorParameterKa_n(i), polySignDq(i));
            }
            else
          SEND_ERROR_STREAM_MSG("Unexpected size of signal dq: "+toString(dq.size()));

//        compute_f(m_tau_star, dq, dq_thr, iter, m_f);
//        compute_g(dq, ddq, ddq_thr, iter, m_g);
//        m_q_des = q.tail<N_JOINTS>() + m_f + m_g;

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
        {
          if(m_activeJoints[i]==false)
            s(i) = 0.0;
          else
            s(i) = m_current_des(i);
        }
//        SEND_MSG("qDes = "+toString(s), MSG_TYPE_DEBUG_STREAM);

//        const int JID = 1;
//        SEND_MSG(toString(iter)+" q   = "+toString(q[6+JID]),MSG_TYPE_DEBUG);
//        SEND_MSG("qDes = "+toString(m_q_des(JID)),MSG_TYPE_DEBUG);
//        SEND_MSG("dq   = "+toString(dq[JID]),MSG_TYPE_DEBUG);
//        SEND_MSG("tau  = "+toString(tau_d(JID)),MSG_TYPE_DEBUG);
        return s;
      }


 DEFINE_SIGNAL_OUT_FUNCTION(controlCurrent, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(current,       m_measuredCurrentSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(current_d,     m_desiredCurrentSOUT(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(kp,            m_KpCurrentSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(ki,            m_KiCurrentSIN(iter));

        m_currentErrIntegral += m_dt * ki.cwiseProduct(current_d-current);
        m_current_star = current_d + kp.cwiseProduct(current_d - current) + m_currentErrIntegral;
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
        {
          if(m_activeJoints[i]==false)
            s(i) = 0.0;
          else if (false) //TODO check saturation
            s(i) = 0.0;
          else
            s(i) = m_current_star(i);
        }
        return s;
      }



      DEFINE_SIGNAL_OUT_FUNCTION(predictedPwm, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,      m_jointsVelocitiesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau,     m_jointsTorquesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_tau,         m_k_tauSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_v,           m_k_vSIN(iter));

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = k_tau(i)*tau(i) + k_v(i)*dq(i);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(predictedPwm_tau, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau,     m_jointsTorquesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_tau,         m_k_tauSIN(iter));

        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = k_tau(i)*tau(i);
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pwm_ff, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tauFF,         m_tauFFSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_tau,         m_k_tauSIN(iter));
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = k_tau[i] * tauFF[i];
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pwm_fb, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau,           m_jointsTorquesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau_d,         m_jointsTorquesDesiredSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(tauFB,         m_tauFBSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_tau,         m_k_tauSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_p,           m_KpTorqueSIN(iter));
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = k_tau[i] * (tauFB[i] + k_p[i]*(tau_d[i]-tau[i]));
        return s;
      }

      DEFINE_SIGNAL_OUT_FUNCTION(pwm_friction, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_v,         m_k_vSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,          m_jointsVelocitiesSIN(iter));
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = k_v[i] * dq[i];
        return s;
      }


      DEFINE_SIGNAL_OUT_FUNCTION(smoothSignDq, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,            m_jointsVelocitiesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(polySignDq,    m_polySignDqSIN(iter));
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);
        for(int i=0; i<N_JOINTS; i++)
          s(i) = motorModel.smoothSign(dq[i], 0.1, polySignDq[i]);
        return s;
      }


//      DEFINE_SIGNAL_OUT_FUNCTION(smoothSignDq, ml::Vector)
//      {
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,      m_jointsVelocitiesSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq_thr,  m_dq_thresholdSIN(iter));

//        if(s.size()!=N_JOINTS)
//          s.resize(N_JOINTS);
//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(dq[i]>dq_thr[i])
//            s(i) = 1.0;
//          else if(dq[i]<-dq_thr[i])
//            s(i) = -1.0;
//          else
//            s(i) = pow(dq[i]/dq_thr[i],3);
//        }
//        return s;
//      }

//      DEFINE_SIGNAL_OUT_FUNCTION(smoothSignDdq, ml::Vector)
//      {
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(ddq,      m_jointsAccelerationsSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(ddq_thr,  m_ddq_thresholdSIN(iter));

//        if(s.size()!=N_JOINTS)
//          s.resize(N_JOINTS);
//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(ddq[i]>ddq_thr[i])
//            s(i) = 1.0;
//          else if(ddq[i]<-ddq_thr[i])
//            s(i) = -1.0;
//          else
//            s(i) = pow(ddq[i]/ddq_thr[i],3);
//        }
//        return s;
//      }

      DEFINE_SIGNAL_OUT_FUNCTION(predictedJointsTorques, ml::Vector)
      {
        EIGEN_CONST_VECTOR_FROM_SIGNAL(pwm,       m_pwmSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq,        m_jointsVelocitiesSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_tau,     m_k_tauSIN(iter));
        EIGEN_CONST_VECTOR_FROM_SIGNAL(k_v,       m_k_vSIN(iter));

        /// k_tau^{-1}*(delta_q - k_v*dq)
        if(s.size()!=N_JOINTS)
          s.resize(N_JOINTS);

        for(int i=0; i<N_JOINTS; i++)
          if(k_tau(i)!=0.0)
            s(i) = (pwm(i) - k_v(i)*dq(i))/k_tau(i);
          else
            s(i) = 0.0;

        return s;
      }

//      void JointTorqueController::compute_f(const VectorXd &tau, const_SigVectorXd &dq, const_SigVectorXd &dq_thr, int iter, VectorXd &f)
//      {
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k1p, m_f_k1pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k2p, m_f_k2pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k3p, m_f_k3pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k1n, m_f_k1nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k2n, m_f_k2nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k3n, m_f_k3nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q1p, m_f_q1pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q2p, m_f_q2pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q3p, m_f_q3pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q1n, m_f_q1nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q2n, m_f_q2nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q3n, m_f_q3nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau1p, m_f_tau1pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau2p, m_f_tau2pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau1n, m_f_tau1nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(tau2n, m_f_tau2nSIN(iter));

//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(dq[i]>dq_thr[i])
//          {
//            f[i] = compute_piecewise_linear(tau[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],tau1p[i],tau2p[i]);
//          }
//          else if(dq[i]<-dq_thr[i])
//          {
//            f[i] = compute_piecewise_linear(tau[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],tau1n[i],tau2n[i]);
//          }
//          else
//          {
//            double fp = compute_piecewise_linear(tau[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],tau1p[i],tau2p[i]);
//            double fn = compute_piecewise_linear(tau[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],tau1n[i],tau2n[i]);
//            double alpha = 0.5*pow(dq[i]/dq_thr[i],3) + 0.5;
//            f[i] = alpha*fp + (1-alpha)*fn;
//          }
//        }
//      }

//      void JointTorqueController::compute_g(const_SigVectorXd &dq, const_SigVectorXd &ddq, const_SigVectorXd &ddq_thr, int iter, VectorXd &g)
//      {
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k1p, m_g_k1pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k2p, m_g_k2pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k3p, m_g_k3pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k1n, m_g_k1nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k2n, m_g_k2nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(k3n, m_g_k3nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q1p, m_g_q1pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q2p, m_g_q2pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q3p, m_g_q3pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q1n, m_g_q1nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q2n, m_g_q2nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(q3n, m_g_q3nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq1p, m_g_dq1pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq2p, m_g_dq2pSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq1n, m_g_dq1nSIN(iter));
//        EIGEN_CONST_VECTOR_FROM_SIGNAL(dq2n, m_g_dq2nSIN(iter));

//        for(int i=0; i<N_JOINTS; i++)
//        {
//          if(ddq[i]>ddq_thr[i])
//          {
//            g[i] = compute_piecewise_linear(dq[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],dq1p[i],dq2p[i]);
//          }
//          else if(ddq[i]<-ddq_thr[i])
//          {
//            g[i] = compute_piecewise_linear(dq[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],dq1n[i],dq2n[i]);
//          }
//          else
//          {
//            double gp = compute_piecewise_linear(dq[i], k1p[i],k2p[i],k3p[i],q1p[i],q2p[i],q3p[i],dq1p[i],dq2p[i]);
//            double gn = compute_piecewise_linear(dq[i], k1n[i],k2n[i],k3n[i],q1n[i],q2n[i],q3n[i],dq1n[i],dq2n[i]);
//            double alpha = 0.5*pow(ddq[i]/ddq_thr[i],3) + 0.5;
//            g[i] = alpha*gp + (1-alpha)*gn;
//          }
//        }
//      }

      double JointTorqueController::compute_piecewise_linear(const double &x, const double &a1, const double &a2, const double &a3, const double &b1,
                                                             const double &b2, const double &b3, const double &x1, const double &x2) const
      {
        if(x<x1)
          return a1*x+b1;
        if(x<x2)
          return a2*x+b2;
        return a3*x+b3;
      }

      bool JointTorqueController::convertJointNameToJointId(const std::string& name, unsigned int& id)
      {
        // Check if the joint name exists
        int jid = JointUtil::get_id_from_name(name);
        if (jid<0)
        {
          SEND_MSG("The specified joint name does not exist", MSG_TYPE_ERROR);
          std::stringstream ss;
          for(map<string, unsigned int>::const_iterator it = JointUtil::name_2_id.begin(); it != JointUtil::name_2_id.end(); it++)
            ss<<it->first<<", ";
          SEND_MSG("Possible joint names are: "+ss.str(), MSG_TYPE_INFO);
          return false;
        }
        id = jid;
        return true;
      }

      void JointTorqueController::display( std::ostream& os ) const
      {
        os << "JointTorqueController "<<getName()<<":\n";
        try
        {
          getProfiler().report_all(3, os);
        }
        catch (ExceptionSignal e) {}
      }

    } // namespace torque_control
  } // namespace sot
} // namespace dynamicgraph
