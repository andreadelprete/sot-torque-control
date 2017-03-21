# -*- coding: utf-8 -*-
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

import numpy as np
from numpy.linalg import norm
from dynamic_graph import plug
from create_entities_utils import NJ
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
from time import sleep

from pinocchio_inv_dyn.simulator import Simulator
from pinocchio_inv_dyn.robot_wrapper import RobotWrapper
import test_balance_ctrl_sim_conf as conf
from pinocchio.utils import zero as mat_zeros

def to_tuple(x):
    return tuple(np.asarray(x).squeeze());

def main(task='', dt=0.001, delay=0.01):
    np.set_printoptions(precision=2, suppress=True);
    dt = conf.dt;
    q0 = conf.q0;
    v0 = conf.v0;
    nv = v0.shape[0];
    
    simulator  = Simulator('hrp2_sim', q0, v0, conf.fMin, conf.mu, dt, conf.model_path, conf.urdfFileName);
    simulator.ENABLE_TORQUE_LIMITS = conf.FORCE_TORQUE_LIMITS;
    simulator.ENABLE_FORCE_LIMITS = conf.ENABLE_FORCE_LIMITS;
    simulator.ENABLE_JOINT_LIMITS = conf.FORCE_JOINT_LIMITS;
    simulator.ACCOUNT_FOR_ROTOR_INERTIAS = conf.ACCOUNT_FOR_ROTOR_INERTIAS;
    simulator.VIEWER_DT = conf.DT_VIEWER;
    simulator.CONTACT_FORCE_ARROW_SCALE = 2e-3;
    simulator.verb=0;
    robot = simulator.r;
    
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");
    ctrl.q.value = to_tuple(simulator.q);
    ctrl.v.value = to_tuple(simulator.v);

    ctrl.com_ref_pos.value = to_tuple(robot.com(q0));
    ctrl.com_ref_pos.value = (0.012, 0.1, 0.81)
    ctrl.com_ref_vel.value = 3*(0.0,);
    ctrl.com_ref_acc.value = 3*(0.0,);
    ctrl.posture_ref_pos.value = to_tuple(q0[7:,0]);
    ctrl.posture_ref_vel.value = NJ*(0.0,);
    ctrl.posture_ref_acc.value = NJ*(0.0,);

    ctrl.contact_normal.value = (0.0, 0.0, 1.0);
    ctrl.contact_points.value = conf.RIGHT_FOOT_CONTACT_POINTS;
    ctrl.f_min.value = conf.fMin;
    ctrl.mu.value = conf.mu[0];
    ctrl.weight_contact_forces.value = (1e2, 1e2, 1e0, 1e3, 1e3, 1e3);
    ctrl.kp_com.value = 3*(conf.kp_com,);
    ctrl.kd_com.value = 3*(conf.kd_com,);
    ctrl.kp_constraints.value = 6*(conf.kp_constr,);
    ctrl.kd_constraints.value = 6*(conf.kd_constr,);
    ctrl.kp_posture.value = 30*(conf.kp_posture,);
    ctrl.kd_posture.value = 30*(conf.kd_posture,);

    ctrl.w_com.value = conf.w_com;
    ctrl.w_forces.value = conf.w_forces;
    ctrl.w_posture.value = conf.w_posture;
    ctrl.w_base_orientation.value = conf.w_base_orientation;
    ctrl.w_torques.value = conf.w_torques;
    
    ctrl.init(dt, conf.urdfFileName);
    ctrl.active_joints.value = conf.active_joints;
    
    t = 0.0;
    dv = mat_zeros(nv);
    for i in range(conf.MAX_TEST_DURATION):
        ctrl.q.value = to_tuple(simulator.q);
        ctrl.v.value = to_tuple(simulator.v);
        ctrl.dv_des.recompute(i);
        dv = np.matrix(ctrl.dv_des.value).T;
        if(norm(dv[6:24]) > 1e-8):
            print "ERROR acceleration of blocked axes is not zero:", norm(dv[6:24]);
        if(i%100==0):
            print "t=%.3f dv=%.1f v=%.1f" % (t, norm(dv), norm(simulator.v));
        simulator.integrateAcc(t, dt, dv, None, None, conf.PLAY_MOTION_WHILE_COMPUTING);
        t += dt;
#        sleep(0.001);
    
    return (simulator, ctrl);
    
if __name__=='__main__':
    (sim, ctrl) = main('none',0.001,0.01);
    pass;
    