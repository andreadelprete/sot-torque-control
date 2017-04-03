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
from dynamic_graph.sot.torque_control.hrp2_device_torque_ctrl import HRP2DeviceTorqueCtrl
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
    np.set_printoptions(precision=3, suppress=True);
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
    
    q0 = np.array(conf.q0_sot);
#    q0[8] -= 1.0;
    device          = create_device(tuple(q0));

    estimator = ForceTorqueEstimator("estimator");
    plug(device.state,     estimator.base6d_encoders);
    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    estimator.gyroscope.value = (0.0, 0.0, 0.0);
    estimator.ftSensRightFoot.value = 6*(0.0,);
    estimator.ftSensLeftFoot.value  = 6*(0.0,);
    estimator.ftSensRightHand.value = 6*(0.0,);
    estimator.ftSensLeftHand.value  = 6*(0.0,);
    estimator.currentMeasure.value = NJ*(0.0,);
    estimator.dqRef.value = NJ*(0.0,);
    estimator.ddqRef.value = NJ*(0.0,);
    estimator.wCurrentTrust.value     = tuple(NJ*[0.0,])
    estimator.saturationCurrent.value = tuple(NJ*[5.0,])
    estimator.motorParameterKt_p.value  = tuple(Kt_p)
    estimator.motorParameterKt_n.value  = tuple(Kt_n)
    estimator.motorParameterKf_p.value  = tuple(Kf_p)
    estimator.motorParameterKf_n.value  = tuple(Kf_n)
    estimator.motorParameterKv_p.value  = tuple(Kv_p)
    estimator.motorParameterKv_n.value  = tuple(Kv_n)
    estimator.motorParameterKa_p.value  = tuple(Ka_p)
    estimator.motorParameterKa_n.value  = tuple(Ka_n)
    estimator.init(dt,delay,delay,delay,delay,delay,True);
    estimator.setUseRawEncoders(True);
    estimator.setUseRefJointVel(True);
    estimator.setUseRefJointAcc(True);
    estimator.setUseFTsensors(False);

    estimator.jointsTorques.recompute(0);
    device.increment(dt);
        
    q = np.matrix(device.state.value).T;
    q_urdf = config_sot_to_urdf(q)
    simulator.viewer.updateRobotConfig(q_urdf);

    print "tau estimator=", np.array(estimator.jointsTorques.value);
    print "tau pinocchio=", robot.bias(q_urdf, conf.v0)[6:,0].T;
    
    return (simulator, estimator);

    
if __name__=='__main__':
    (sim, estimator) = main();
    pass;
    
