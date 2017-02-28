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


#define INPUT_SIGNALS     m_base6d_encodersSIN
#define OUTPUT_SIGNALS    m_base6dFromFoot_encodersSOUT

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
            ,CONSTRUCT_SIGNAL_IN( base6d_encoders,         ml::Vector)
            ,CONSTRUCT_SIGNAL_OUT(base6dFromFoot_encoders, ml::Vector, INPUT_SIGNALS)      
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
        } 
        catch (const std::exception& e) 
        { 
          std::cout << e.what();
          return SEND_MSG("Init failed: Could load URDF :" + urdfFile, MSG_TYPE_ERROR);
        }
        m_data = new se3::Data(m_model);
        cout<<m_model;  
        m_initSucceeded = true;
      }

      //~ void FreeFlyerLocator::resetIntegral()
      //~ {
        //~ m_e_integral.setZero(N_JOINTS);
      //~ }

      /* ------------------------------------------------------------------- */
      /* --- SIGNALS ------------------------------------------------------- */
      /* ------------------------------------------------------------------- */

      DEFINE_SIGNAL_OUT_FUNCTION(base6dFromFoot_encoders,ml::Vector)
      {
        if(!m_initSucceeded)
        {
          SEND_WARNING_STREAM_MSG("Cannot compute signal freeflyer before initialization!");
          return s;
        }
        
        //~ getProfiler().start(PROFILE_FREE_FLYER_COMPUTATION);
        //~ {
          EIGEN_CONST_VECTOR_FROM_SIGNAL(q, m_base6d_encodersSIN(iter));     //n+6
          assert(q.size()==N_JOINTS+6     && "Unexpected size of signal base6d_encoder");
          Eigen::VectorXd q_pin(Eigen::VectorXd::Zero(m_model.nq));
          
          /* fill q_pin with pinocchio joint order (Very specific to HRP2 Robot)*/
          
          // GEPETTO VIEWER Free flyer 0-6, CHEST HEAD 7-10, LARM 11-17, RARM 18-24, LLEG 25-30, RLEG 31-36
          // ROBOT VIEWER # Free flyer0-5, RLEG 6-11, LLEG 12-17, CHEST HEAD 18-21, RARM 22-28, LARM 29-35
          q_pin[6 ]= 1.; // for quaternion
          
          q_pin[7 ]=q[18]; //HEAD
          q_pin[8 ]=q[19];
          
          q_pin[9 ]=q[20]; //CHEST
          q_pin[10]=q[21];
          
          q_pin[11]=q[29]; //LARM
          q_pin[12]=q[30];
          q_pin[13]=q[31];
          q_pin[14]=q[32];
          q_pin[15]=q[33];
          q_pin[16]=q[34];
          q_pin[17]=q[35];

          q_pin[18]=q[22]; //RARM
          q_pin[19]=q[23];
          q_pin[20]=q[24];
          q_pin[21]=q[25];
          q_pin[22]=q[26];
          q_pin[23]=q[27];
          q_pin[24]=q[28];
          
          q_pin[25]=q[12]; //LLEG
          q_pin[26]=q[13];
          q_pin[27]=q[14];
          q_pin[28]=q[15];
          q_pin[29]=q[16];
          q_pin[30]=q[17];
          
          q_pin[31]=q[6 ]; //RLEG
          q_pin[32]=q[7 ];
          q_pin[33]=q[8 ];
          q_pin[34]=q[9 ];
          q_pin[35]=q[10];
          q_pin[36]=q[11];
          /* Compute kinematic and return q with freeflyer */
          forwardKinematics(m_model,*m_data,q_pin);
          const se3::SE3 & iMo = m_data->oMi[30].inverse(); 
          if(s.size()!=6)
             s.resize(6);
          const Eigen::AngleAxisd aa(iMo.rotation());
          Eigen::VectorXd freeflyer(Eigen::VectorXd::Zero(6));
          Eigen::VectorXd q_out(Eigen::VectorXd::Zero(N_JOINTS+6));
          q_out = q;
          freeflyer << iMo.translation(), aa.axis() * aa.angle();
          freeflyer[2]+=0.105; // due to distance from ankle to ground on HRP2
          q_out(0) = freeflyer(0);
          q_out(1) = freeflyer(1);
          q_out(2) = freeflyer(2);
          q_out(3) = freeflyer(3);
          q_out(4) = freeflyer(4);
          q_out(5) = freeflyer(5);

          EIGEN_VECTOR_TO_VECTOR(q_out,s);
        //~ }
        //~ getProfiler().stop(PROFILE_FREE_FLYER_COMPUTATION);

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

