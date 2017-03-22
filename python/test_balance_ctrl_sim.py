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
from dynamic_graph.sot.core.robot_simu import RobotSimu
from create_entities_utils import *
from time import sleep

from pinocchio_inv_dyn.simulator import Simulator
from pinocchio_inv_dyn.robot_wrapper import RobotWrapper
import test_balance_ctrl_sim_conf as conf
from pinocchio.utils import zero as mat_zeros
from pinocchio import Quaternion
from pinocchio.rpy import rpyToMatrix

def to_tuple(x):
    return tuple(np.asarray(x).squeeze());

def config_sot_to_urdf(q):
    # GEPETTO VIEWER Free flyer 0-6, CHEST HEAD 7-10, LARM 11-17, RARM 18-24, LLEG 25-30, RLEG 31-36
    # ROBOT VIEWER # Free flyer0-5, RLEG 6-11, LLEG 12-17, CHEST HEAD 18-21, RARM 22-28, LARM 29-35
    qUrdf = mat_zeros(37);
    qUrdf[:3,0] = q[:3,0];
    quatMat = rpyToMatrix(q[3:6,0]);
    quatVec = Quaternion(quatMat);
    qUrdf[3:7,0]   = quatVec.coeffs();
    qUrdf[7:11,0]  = q[18:22,0]; # chest-head
    qUrdf[11:18,0] = q[29:,0]; # larm
    qUrdf[18:25,0] = q[22:29,0]; # rarm
    qUrdf[25:31,0] = q[12:18,0]; # lleg
    qUrdf[31:,0]   = q[6:12,0]; # rleg
    return qUrdf;

def create_device(q=None):
    device = RobotSimu("device");
    device.setControlInputType('acceleration');
    if(q==None):
        q = (0,0,0.7)+(NJ+3)*(0.0,);
    ddq_des = (NJ+6)*(0.0,);
    device.resize(NJ+6);
    device.set(q);
    device.control.value = ddq_des;
    device.increment(0.001);
    return device;
    
def main(dt=0.001, delay=0.01):
    np.set_printoptions(precision=2, suppress=True);
    COM_DES_1 = (0.012, 0.1, 0.81);
    COM_DES_2 = (0.012, -0.1, 0.81);
    dt = conf.dt;
    q0 = conf.q0_urdf;
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
    
    device          = create_device(conf.q0_sot);
#    ff_locator      = create_free_flyer_locator(device, conf.urdfFileName);
    traj_gen        = create_trajectory_generator(device, dt);
#    estimator       = create_estimator(device, dt, delay, traj_gen);
#    torque_ctrl     = create_torque_controller(device, estimator);
#    pos_ctrl        = create_position_controller(device, estimator, dt, traj_gen);
    
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");
    plug(device.state,                 ctrl.q);
    ctrl.v.value = 36*(0.0,);
#    plug(estimator.jointsVelocities,        ctrl.v);
    plug(traj_gen.q,                        ctrl.posture_ref_pos);
    plug(traj_gen.dq,                       ctrl.posture_ref_vel);
    plug(traj_gen.ddq,                      ctrl.posture_ref_acc);
#    plug(estimator.contactWrenchRightSole,  ctrl.wrench_right_foot);
#    plug(estimator.contactWrenchLeftSole,   ctrl.wrench_left_foot);
#    plug(ctrl.tau_des,                      torque_ctrl.jointsTorquesDesired);
#    plug(ctrl.tau_des,                      estimator.tauDes);

    ctrl.com_ref_pos.value = to_tuple(robot.com(q0));
    ctrl.com_ref_pos.value = COM_DES_1;
    ctrl.com_ref_vel.value = 3*(0.0,);
    ctrl.com_ref_acc.value = 3*(0.0,);

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
    
#    ctrl_manager    = create_ctrl_manager(device, torque_ctrl, pos_ctrl, ctrl, estimator, dt);
#    plug(device.velocity,       torque_ctrl.jointsVelocities);    
    plug(device.velocity,       ctrl.v);
    plug(ctrl.dv_des,           device.control);
    t = 0.0;
    v = mat_zeros(nv);
    dv = mat_zeros(nv);
    x_rf = robot.framePosition(robot.model.getFrameId('RLEG_JOINT5')).translation;
    x_lf = robot.framePosition(robot.model.getFrameId('LLEG_JOINT5')).translation;
    for i in range(conf.MAX_TEST_DURATION):
#        if(norm(dv[6:24]) > 1e-8):
#            print "ERROR acceleration of blocked axes is not zero:", norm(dv[6:24]);
        device.increment(dt);
        
        if(i==1500):
            ctrl.com_ref_pos.value = COM_DES_2;
        if(i%10==0):
            q = np.matrix(device.state.value).T;
            q_urdf = config_sot_to_urdf(q);
            simulator.viewer.updateRobotConfig(q_urdf);
            ctrl.f_des_right_foot.recompute(i);
            ctrl.f_des_left_foot.recompute(i);
            f_rf = np.matrix(ctrl.f_des_right_foot.value).T
            f_lf = np.matrix(ctrl.f_des_left_foot.value).T
            simulator.updateContactForcesInViewer(['rf', 'lf'], 
                                                  [x_rf, x_lf], 
                                                  [f_rf, f_lf]);
            ctrl.com.recompute(i);
            com = np.matrix(ctrl.com.value).T
            com[2,0] = 0.0;
            simulator.updateComPositionInViewer(com);
        if(i%100==0):
            ctrl.com.recompute(i);
            com = np.matrix(ctrl.com.value).T
            v = np.matrix(device.velocity.value).T;
            dv = np.matrix(ctrl.dv_des.value).T;
            print "t=%.3f dv=%.1f v=%.1f com=" % (t, norm(dv), norm(v)), com.T,
            print "zmp_lf", f_lf[3:5].T/f_lf[2,0],
            print "zmp_rf", f_rf[3:5].T/f_rf[2,0];
            
        if(i==2):
            ctrl_manager = ControlManager("ctrl_man");
            ctrl_manager.resetProfiler();
        t += dt;
    
    return (simulator, ctrl);

def main_old(task='', dt=0.001, delay=0.01):
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
    (sim, ctrl) = main();
    pass;
    