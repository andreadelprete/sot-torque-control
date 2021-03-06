/*
 * Copyright 2014, Andrea Del Prete, LAAS-CNRS
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


#ifndef __sot_torque_control_metapod_helper_H__
#define __sot_torque_control_metapod_helper_H__

/* Metapod */
# include <metapod/tools/common.hh>
# include <boost/fusion/sequence.hpp>

namespace metapod {

  /**
   * Apply a spatial transformation to a given matrix.
   * It can be used to transform the Jacobian of a frame <a> into the Jacobian
   * of another frame <b> given the transformation between <a> and <b>, that is
   * b_X_a:
   * J_b = b_X_a * J_a
   * This operation is equivalent to:
   *   X.toMatrix()*A
   * but more efficient because it exploits the fact that the rotation matrix
   * is an identity.
   */
  template<typename _Scalar, int _Cols, int _Options, int _MaxRows, int _MaxCols>
  void applyTransformToMatrix(
      const Spatial::TransformT<_Scalar, Spatial::RotationMatrixIdentityTpl<_Scalar> > &X,
      Eigen::Matrix< _Scalar, 6, _Cols, _Options, _MaxRows, _MaxCols > &A)
  {
    A.template bottomRows<3>() -= Spatial::skew<_Scalar>(X.r()) * A.template topRows<3>();
  }


  /**
   * Update parent body kinematics using data from child node body and joint.
   * The child node needs to have already computed:
   * - sXp: transform from parent to current node (computed in jcalc and bcalc)
   * - vj: velocity generated by joint's velocity (computed in jcalc)
   * - cj: product of apparent derivative of S and joint velocity (computed in jcalc)
   * - vi: velocity of the body (computed in rnea)
   * - ai: acceleration of the body (computed in rnea)
   */
  template< typename Robot, int node_id, int parent_id >
  struct update_kinematics_backward
  {
    typedef Spatial::MotionTpl<typename Robot::RobotFloatType> Motion;
    typedef typename Nodes<Robot, node_id>::type Node;
    typedef typename Nodes<Robot, parent_id>::type Parent;

    static void run(
        Robot & robot,
        const Eigen::Matrix< typename Robot::RobotFloatType, Node::Joint::NBDOF, 1 > & ddqi)
    {
      Node& node = boost::fusion::at_c<node_id>(robot.nodes);
      Parent& parent = boost::fusion::at_c<parent_id>(robot.nodes);

      // cannot compute iX0 when propagating backward because iXo
      // is not known for the initial body
      // vλ(i) = iXλ(i)^-1 * (vi - vj)
      // aλ(i) = iXλ(i)^-1 * (ai - Si * ddqi + cj - vi x vj)
      parent.body.vi = node.sXp.applyInv(node.body.vi + (-node.joint.vj));
      parent.body.ai = node.sXp.applyInv(sum(node.body.ai,
                                             - Motion(node.joint.S.S() * ddqi),
                                             - node.joint.cj,
                                             - (node.body.vi^node.joint.vj)));
    }
  };

}

#endif // __sot_torque_control_metapod_helper_H__
