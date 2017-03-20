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

#include <sot/torque_control/hrp2-common.hh>
#include <sot/core/debug.hh>
#include <dynamic-graph/factory.h>


namespace dynamicgraph
{
  namespace sot
  {
    namespace torque_control
    {
      namespace dg = ::dynamicgraph;
      using namespace dg;
      using namespace dg::command;

      bool config_urdf_to_sot(const Eigen::VectorXd & q_urdf, Eigen::VectorXd & q_sot)
      {
        return true;
      }

      bool config_sot_to_urdf(const Eigen::VectorXd & q_sot, Eigen::VectorXd & q_urdf)
      {
        q_urdf[0 ]=q_sot[0]; //BASE
        q_urdf[1 ]=q_sot[1];
        q_urdf[2 ]=q_sot[2];

        q_urdf[7 ]=q_sot[18]; //HEAD
        q_urdf[8 ]=q_sot[19];

        q_urdf[9 ]=q_sot[20]; //CHEST
        q_urdf[10]=q_sot[21];

        q_urdf[11]=q_sot[29]; //LARM
        q_urdf[12]=q_sot[30];
        q_urdf[13]=q_sot[31];
        q_urdf[14]=q_sot[32];
        q_urdf[15]=q_sot[33];
        q_urdf[16]=q_sot[34];
        q_urdf[17]=q_sot[35];

        q_urdf[18]=q_sot[22]; //RARM
        q_urdf[19]=q_sot[23];
        q_urdf[20]=q_sot[24];
        q_urdf[21]=q_sot[25];
        q_urdf[22]=q_sot[26];
        q_urdf[23]=q_sot[27];
        q_urdf[24]=q_sot[28];

        q_urdf[25]=q_sot[12]; //LLEG
        q_urdf[26]=q_sot[13];
        q_urdf[27]=q_sot[14];
        q_urdf[28]=q_sot[15];
        q_urdf[29]=q_sot[16];
        q_urdf[30]=q_sot[17];

        q_urdf[31]=q_sot[6 ]; //RLEG
        q_urdf[32]=q_sot[7 ];
        q_urdf[33]=q_sot[8 ];
        q_urdf[34]=q_sot[9 ];
        q_urdf[35]=q_sot[10];
        q_urdf[36]=q_sot[11];
        return true;
      }

      bool velocity_urdf_to_sot(const Eigen::VectorXd & v_urdf, Eigen::VectorXd & v_sot)
      {
        return true;
      }

      bool velocity_sot_to_urdf(const Eigen::VectorXd & v_sot, Eigen::VectorXd & v_urdf)
      {
        return true;
      }

      
    } // namespace torquecontrol
  } // namespace sot
} // namespace dynamicgraph

