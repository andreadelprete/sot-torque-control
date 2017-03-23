# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from dynamic_graph.sot.torque_control.force_torque_estimator import ForceTorqueEstimator
from dynamic_graph.sot.torque_control.joint_torque_controller import JointTorqueController
from dynamic_graph.sot.torque_control.joint_trajectory_generator import JointTrajectoryGenerator
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.inverse_dynamics_controller import InverseDynamicsController
from dynamic_graph.sot.torque_control.inverse_dynamics_balance_controller import InverseDynamicsBalanceController
from dynamic_graph.sot.torque_control.admittance_controller import AdmittanceController
from dynamic_graph.sot.torque_control.position_controller import PositionController
from dynamic_graph.tracer_real_time import TracerRealTime
from hrp2_motors_parameters import NJ
from hrp2_motors_parameters import *
from hrp2_joint_pos_ctrl_gains import *
import numpy as np

def create_free_flyer_locator(device, urdf, dynamic=None):
    from dynamic_graph.sot.torque_control.free_flyer_locator import FreeFlyerLocator
    ff_locator = FreeFlyerLocator("ffLocator");
    plug(device.robotState, ff_locator.base6d_encoders);
    if(dynamic!=None):
        plug(ff_locator.base6dFromFoot_encoders, dynamic.position);
    ff_locator.init(urdf);
    return ff_locator;
    
def create_flex_estimator(robot, dt=0.001):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
    flex_est = HRP2ModelBaseFlexEstimatorIMUForce(robot, useMocap=False, dt=dt);
    flex_est.setOn(False)
    flex_est.interface.setExternalContactPresence(False)
    #flex_est.enabledContacts_lf_rf_lh_rh.value=(1,1,0,0)
    flex_est.leftFootVelocity.sin2.value = 36*(0.0,)
    flex_est.rightFootVelocity.sin2.value = 36*(0.0,)
    flex_est.inputVel.sin2.value = 36*(0.0,)
    flex_est.DCom.sin2.value = 36*(0.0,)
    robot.device.after.addSignal('flextimator.state');
    return flex_est;
    
def create_floatingBase(flex_est,ff_locator):
    from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import FromLocalToGLobalFrame 
    floatingBase = FromLocalToGLobalFrame(flex_est, "FloatingBase")
    plug(ff_locator.freeflyer_aa, floatingBase.sinPos)
    return floatingBase
    
def create_position_controller(device, estimator, dt=0.001, traj_gen=None):
    posCtrl = PositionController('pos_ctrl')
    posCtrl.Kp.value = tuple(kp_pos);
    posCtrl.Kd.value = tuple(kd_pos);
    posCtrl.Ki.value = tuple(ki_pos);
    posCtrl.dqRef.value = 30*(0.0,);
    plug(device.robotState,             posCtrl.base6d_encoders);    
    plug(estimator.jointsVelocities,    posCtrl.jointsVelocities);
    plug(posCtrl.pwmDes,                device.control);
    if(traj_gen!=None):
        plug(traj_gen.q,       posCtrl.qRef);
    posCtrl.init(dt);
    return posCtrl;

def create_trajectory_generator(device, dt=0.001):
    jtg = JointTrajectoryGenerator("jtg");
    plug(device.robotState,             jtg.base6d_encoders);
    jtg.init(dt);
    return jtg;

def create_estimator(device, dt, delay, traj_gen=None):
    estimator = ForceTorqueEstimator("estimator");

    plug(device.robotState,     estimator.base6d_encoders);
    plug(device.accelerometer,  estimator.accelerometer);
    plug(device.gyrometer,      estimator.gyroscope);
    plug(device.forceRLEG,      estimator.ftSensRightFoot);
    plug(device.forceLLEG,      estimator.ftSensLeftFoot);
    plug(device.forceRARM,      estimator.ftSensRightHand);
    plug(device.forceLARM,      estimator.ftSensLeftHand);
    plug(device.currents,       estimator.currentMeasure);
    if(traj_gen!=None):
        plug(traj_gen.dq,       estimator.dqRef);
        plug(traj_gen.ddq,      estimator.ddqRef);
    estimator.wCurrentTrust.value     = tuple(NJ*[0.5,])
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
    
    return estimator;
        
def create_torque_controller(device, estimator, dt=0.001):
    torque_ctrl = JointTorqueController("jtc");
    plug(device.robotState,             torque_ctrl.base6d_encoders);
    plug(estimator.jointsVelocities,    torque_ctrl.jointsVelocities);
    plug(estimator.jointsAccelerations, torque_ctrl.jointsAccelerations);
    plug(estimator.jointsTorques,       torque_ctrl.jointsTorques);
    plug(estimator.currentFiltered,               torque_ctrl.measuredCurrent);
    torque_ctrl.jointsTorquesDesired.value = NJ*(0.0,);
    torque_ctrl.KpTorque.value = tuple(k_p_torque);
    torque_ctrl.KiTorque.value = NJ*(0.0,);
    torque_ctrl.KpCurrent.value = tuple(k_p_current);
    torque_ctrl.KiCurrent.value = NJ*(0.0,);
    torque_ctrl.k_tau.value = tuple(k_tau);
    torque_ctrl.k_v.value   = tuple(k_v);
    torque_ctrl.frictionCompensationPercentage.value = NJ*(0.8,); # 80%

    torque_ctrl.motorParameterKt_p.value  = tuple(Kt_p)
    torque_ctrl.motorParameterKt_n.value  = tuple(Kt_n)
    torque_ctrl.motorParameterKf_p.value  = tuple(Kf_p)
    torque_ctrl.motorParameterKf_n.value  = tuple(Kf_n)
    torque_ctrl.motorParameterKv_p.value  = tuple(Kv_p)
    torque_ctrl.motorParameterKv_n.value  = tuple(Kv_n)
    torque_ctrl.motorParameterKa_p.value  = tuple(Ka_p)
    torque_ctrl.motorParameterKa_n.value  = tuple(Ka_n)
    torque_ctrl.polySignDq.value          = NJ*(3,); 
    torque_ctrl.init(dt);
    return torque_ctrl;
   
def create_balance_controller(device, floatingBase, estimator, torque_ctrl, traj_gen, urdfFileName, dt=0.001):
    ctrl = InverseDynamicsBalanceController("invDynBalCtrl");

    from dynamic_graph.sot.core import Stack_of_vector
    base6d_encoders = Stack_of_vector('base6d_encoders');
    plug(floatingBase.soutPos, base6d_encoders.sin1);
    base6d_encoders.selec1(0,6);
    plug(device.robotState,    base6d_encoders.sin2);
    base6d_encoders.selec2(6,36);
    plug(base6d_encoders.sout,                 ctrl.q);

    ctrl.v.value = 36*(0.0,);
#    plug(estimator.jointsVelocities,        ctrl.v);
    plug(traj_gen.q,                        ctrl.posture_ref_pos);
    plug(traj_gen.dq,                       ctrl.posture_ref_vel);
    plug(traj_gen.ddq,                      ctrl.posture_ref_acc);
#    plug(estimator.contactWrenchRightSole,  ctrl.wrench_right_foot);
#    plug(estimator.contactWrenchLeftSole,   ctrl.wrench_left_foot);
    plug(ctrl.tau_des,                      torque_ctrl.jointsTorquesDesired);
    plug(ctrl.tau_des,                      estimator.tauDes);

    import test_balance_ctrl_sim_conf as conf
    ctrl.com_ref_pos.value = conf.COM_DES;
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
    ctrl.kp_pos.value = 30*(conf.kp_pos,);
    ctrl.kd_pos.value = 30*(conf.kd_pos,);

    ctrl.w_com.value = conf.w_com;
    ctrl.w_forces.value = conf.w_forces;
    ctrl.w_posture.value = conf.w_posture;
    ctrl.w_base_orientation.value = conf.w_base_orientation;
    ctrl.w_torques.value = conf.w_torques;
    
    ctrl.init(dt, urdfFileName);
    
    return ctrl;
    
def create_inverse_dynamics(device, estimator, torque_ctrl, traj_gen, dt=0.001):
    inv_dyn_ctrl = InverseDynamicsController("inv_dyn");
    plug(device.robotState,             inv_dyn_ctrl.base6d_encoders);
    plug(estimator.jointsVelocities,    inv_dyn_ctrl.jointsVelocities);
    plug(traj_gen.q,                    inv_dyn_ctrl.qRef);
    plug(traj_gen.dq,                   inv_dyn_ctrl.dqRef);
    plug(traj_gen.ddq,                  inv_dyn_ctrl.ddqRef);
    plug(estimator.contactWrenchRightSole,   inv_dyn_ctrl.fRightFoot);
    plug(estimator.contactWrenchLeftSole,    inv_dyn_ctrl.fLeftFoot);
    plug(estimator.contactWrenchRightHand,   inv_dyn_ctrl.fRightHand);
    plug(estimator.contactWrenchLeftHand,    inv_dyn_ctrl.fLeftHand);
    plug(traj_gen.fRightFoot,           inv_dyn_ctrl.fRightFootRef);
    plug(traj_gen.fLeftFoot,            inv_dyn_ctrl.fLeftFootRef);
    plug(traj_gen.fRightHand,           inv_dyn_ctrl.fRightHandRef);
    plug(traj_gen.fLeftHand,            inv_dyn_ctrl.fLeftHandRef);
    plug(estimator.baseAngularVelocity, inv_dyn_ctrl.baseAngularVelocity);
    plug(estimator.baseAcceleration,    inv_dyn_ctrl.baseAcceleration);
    plug(inv_dyn_ctrl.tauDes,           torque_ctrl.jointsTorquesDesired);
    plug(inv_dyn_ctrl.tauFF,            torque_ctrl.tauFF);
    plug(inv_dyn_ctrl.tauFB,            torque_ctrl.tauFB);
    plug(inv_dyn_ctrl.tauDes,           estimator.tauDes);
    plug(estimator.dynamicsError,       inv_dyn_ctrl.dynamicsError);
    
    inv_dyn_ctrl.dynamicsErrorGain.value = (NJ+6)*(0.0,);
    inv_dyn_ctrl.Kp.value = tuple(k_s); # joint proportional gains
    inv_dyn_ctrl.Kd.value = tuple(k_d); # joint derivative gains
    inv_dyn_ctrl.Kf.value = tuple(k_f); # force proportional gains
    inv_dyn_ctrl.Ki.value = tuple(k_i); # force integral gains
    inv_dyn_ctrl.controlledJoints.value = NJ*(1.0,);
    inv_dyn_ctrl.init(dt);
    return inv_dyn_ctrl;
        
def create_ctrl_manager(device, torque_ctrl, pos_ctrl, inv_dyn, estimator, dt=0.001):
    ctrl_manager = ControlManager("ctrl_man");
    plug(device.robotState,                  ctrl_manager.base6d_encoders);

    plug(torque_ctrl.predictedJointsTorques, ctrl_manager.tau_predicted);
    plug(estimator.jointsTorques,            ctrl_manager.tau);
    ctrl_manager.max_tau.value = tuple(tau_max);
    ctrl_manager.percentageDriverDeadZoneCompensation.value = NJ*(0.5,);
    ctrl_manager.signWindowsFilterSize.value = NJ*(2,);
    ctrl_manager.bemfFactor.value = NJ*(0.0,);
    #ctrl_manager.bemfFactor.value = tuple(Kpwm*0.1);
    plug(ctrl_manager.pwmDesSafe,       device.control);
    plug(ctrl_manager.pwmDes,           torque_ctrl.pwm);
    ctrl_manager.addCtrlMode("pos");
    ctrl_manager.addCtrlMode("torque");    
    plug(estimator.jointsVelocities,    ctrl_manager.dq);
    plug(torque_ctrl.controlCurrent,    ctrl_manager.ctrl_torque);
    plug(pos_ctrl.pwmDes,               ctrl_manager.ctrl_pos);
    plug(ctrl_manager.joints_ctrl_mode_torque,  inv_dyn.active_joints);
    ctrl_manager.setCtrlMode("all", "pos");
    ctrl_manager.init(dt);
    return ctrl_manager;

def create_admittance_ctrl(device, estimator, ctrl_manager, traj_gen, dt=0.001):
    admit_ctrl = AdmittanceController("adm_ctrl");
    plug(device.robotState,             admit_ctrl.base6d_encoders);
    plug(estimator.jointsVelocities,    admit_ctrl.jointsVelocities);
    plug(estimator.contactWrenchRightSole,   admit_ctrl.fRightFoot);
    plug(estimator.contactWrenchLeftSole,    admit_ctrl.fLeftFoot);
    plug(estimator.contactWrenchRightHand,   admit_ctrl.fRightHand);
    plug(estimator.contactWrenchLeftHand,    admit_ctrl.fLeftHand);
    plug(traj_gen.fRightFoot,           admit_ctrl.fRightFootRef);
    plug(traj_gen.fLeftFoot,            admit_ctrl.fLeftFootRef);
    plug(traj_gen.fRightHand,           admit_ctrl.fRightHandRef);
    plug(traj_gen.fLeftHand,            admit_ctrl.fLeftHandRef);
    
    admit_ctrl.damping.value = 4*(0.05,);
    admit_ctrl.Kd.value = NJ*(0,);
    kf = -0.0005;
    km = -0.008;
    admit_ctrl.Kf.value = 3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,)+3*(kf,)+3*(km,);
    
    ctrl_manager.addCtrlMode("adm");
    plug(admit_ctrl.qDes,                   ctrl_manager.ctrl_adm);
    plug(ctrl_manager.joints_ctrl_mode_adm, admit_ctrl.controlledJoints);
    
    admit_ctrl.init(dt);
    return admit_ctrl;

def create_ros_topics(robot=None, estimator=None, torque_ctrl=None, traj_gen=None, ctrl_manager=None, inv_dyn=None, adm_ctrl=None, ff_locator=None, floatingBase=None):
    from dynamic_graph.ros import RosImport
    ros = RosImport('rosImport');
    if(robot!=None):
        ros.add('vector', 'robotState_ros',     'robotState');
        ros.add('vector', 'gyrometer_ros',      'gyrometer');
        ros.add('vector', 'accelerometer_ros',  'accelerometer');
        ros.add('vector', 'forceRLEG_ros',      'forceRLEG');
        ros.add('vector', 'forceLLEG_ros',      'forceLLEG');
        ros.add('vector', 'currents_ros',      'currents');
        ros.add('vector', 'forceRARM_ros',      'forceRARM');
        ros.add('vector', 'forceLARM_ros',      'forceLARM');
        plug(robot.device.robotState,    ros.robotState_ros);        
        plug(robot.device.gyrometer,     ros.gyrometer_ros);
        plug(robot.device.accelerometer, ros.accelerometer_ros);
        plug(robot.device.forceRLEG,     ros.forceRLEG_ros);
        plug(robot.device.forceLLEG,     ros.forceLLEG_ros);
        plug(robot.device.currents,     ros.currents_ros);
        plug(robot.device.forceRARM,     ros.forceRARM_ros);
        plug(robot.device.forceLARM,     ros.forceLARM_ros);


        
#        robot.device.after.addSignal('rosImport.trigger');
        robot.device.after.addDownsampledSignal('rosImport.trigger',30);
    if(estimator!=None):
#        ros.add('vector', 'estimator_jointsPositions_ros',          'estimator_jointsPositions');
        ros.add('vector', 'estimator_jointsVelocities_ros',         'estimator_jointsVelocities');
        ros.add('vector', 'estimator_jointsAccelerations_ros',      'estimator_jointsAccelerations');
#        ros.add('vector', 'estimator_torsoAcceleration_ros',        'estimator_torsoAcceleration');
#        ros.add('vector', 'estimator_torsoAngularVelocity_ros',     'estimator_torsoAngularVelocity');
        ros.add('vector', 'estimator_contactWrenchLeftSole_ros',    'estimator_contactWrenchLeftSole');
        ros.add('vector', 'estimator_contactWrenchRightSole_ros',   'estimator_contactWrenchRightSole');
        ros.add('vector', 'estimator_ftSensRightFootPrediction_ros', 'estimator_ftSensRightFootPrediction');
#        ros.add('vector', 'estimator_contactWrenchLeftHand_ros',    'estimator_contactWrenchLeftHand');
#        ros.add('vector', 'estimator_contactWrenchRightHand_ros',   'estimator_contactWrenchRightHand');
#        ros.add('vector', 'estimator_contactWrenchBody_ros',        'estimator_contactWrenchBody');
        ros.add('vector', 'estimator_jointsTorques_ros',                 'estimator_jointsTorques');
        ros.add('vector', 'estimator_jointsTorquesFromInertiaModel_ros', 'estimator_jointsTorquesFromInertiaModel');
        ros.add('vector', 'estimator_jointsTorquesFromMotorModel_ros',   'estimator_jointsTorquesFromMotorModel');
        ros.add('vector', 'estimator_currentFiltered_ros',               'estimator_currentFiltered');
        ros.add('vector', 'estimator_dynamicsError_ros',                 'estimator_dynamicsError');
#        plug(estimator.jointsPositions,         ros.estimator_jointsPositions_ros);
        plug(estimator.jointsVelocities,        ros.estimator_jointsVelocities_ros);
        plug(estimator.jointsAccelerations,     ros.estimator_jointsAccelerations_ros);
#        plug(estimator.torsoAcceleration,       ros.estimator_torsoAcceleration_ros);
#        plug(estimator.torsoAngularVelocity,    ros.estimator_torsoAngularVelocity_ros);
        plug(estimator.contactWrenchLeftSole,   ros.estimator_contactWrenchLeftSole_ros);
        plug(estimator.contactWrenchRightSole,  ros.estimator_contactWrenchRightSole_ros);
        plug(estimator.ftSensRightFootPrediction,  ros.estimator_ftSensRightFootPrediction_ros);
#        plug(estimator.contactWrenchLeftHand,   ros.estimator_contactWrenchLeftHand_ros);
#        plug(estimator.contactWrenchRightHand,  ros.estimator_contactWrenchRightHand_ros);
#        plug(estimator.contactWrenchBody,       ros.estimator_contactWrenchBody_ros);
        plug(estimator.jointsTorques,                     ros.estimator_jointsTorques_ros);
        plug(estimator.jointsTorquesFromInertiaModel,     ros.estimator_jointsTorquesFromInertiaModel_ros);
        plug(estimator.jointsTorquesFromMotorModel,       ros.estimator_jointsTorquesFromMotorModel_ros);
        plug(estimator.currentFiltered,                   ros.estimator_currentFiltered_ros);
        plug(estimator.dynamicsError,                      ros.estimator_dynamicsError_ros);
        robot.device.after.addSignal('estimator.contactWrenchRightFoot')
    if(torque_ctrl!=None):
        ros.add('vector', 'torque_ctrl_predictedPwm_ros',           'torque_ctrl_predictedPwm');
        ros.add('vector', 'torque_ctrl_predictedPwm_tau_ros',       'torque_ctrl_predictedPwm_tau');
        ros.add('vector', 'torque_ctrl_pwm_ff_ros',                 'torque_ctrl_pwm_ff');
        ros.add('vector', 'torque_ctrl_pwm_fb_ros',                 'torque_ctrl_pwm_fb');
        ros.add('vector', 'torque_ctrl_pwm_friction_ros',           'torque_ctrl_pwm_friction');
        ros.add('vector', 'torque_ctrl_smoothSignDq_ros',           'torque_ctrl_smoothSignDq');
        ros.add('vector', 'torque_ctrl_predictedJointsTorques_ros', 'torque_ctrl_predictedJointsTorques');
        ros.add('vector', 'torque_ctrl_controlCurrent_ros',         'torque_ctrl_controlCurrent');
        ros.add('vector', 'torque_ctrl_desiredCurrent_ros',         'torque_ctrl_desiredCurrent');
        plug(torque_ctrl.predictedPwm,            ros.torque_ctrl_predictedPwm_ros);
        plug(torque_ctrl.predictedPwm_tau,        ros.torque_ctrl_predictedPwm_tau_ros);
        plug(torque_ctrl.pwm_ff,                  ros.torque_ctrl_pwm_ff_ros);
        plug(torque_ctrl.pwm_fb,                  ros.torque_ctrl_pwm_fb_ros);
        plug(torque_ctrl.pwm_friction,            ros.torque_ctrl_pwm_friction_ros);
        plug(torque_ctrl.smoothSignDq,            ros.torque_ctrl_smoothSignDq_ros);
        plug(torque_ctrl.predictedJointsTorques,  ros.torque_ctrl_predictedJointsTorques_ros);
        plug(torque_ctrl.controlCurrent,          ros.torque_ctrl_controlCurrent_ros);
        plug(torque_ctrl.desiredCurrent,          ros.torque_ctrl_desiredCurrent_ros);

    if(traj_gen!=None):
        ros.add('vector', 'traj_gen_q_ros',             'traj_gen_q');
        ros.add('vector', 'traj_gen_dq_ros',            'traj_gen_dq');
        ros.add('vector', 'traj_gen_ddq_ros',           'traj_gen_ddq');
        ros.add('vector', 'traj_gen_fRightFoot_ros',    'traj_gen_fRightFoot');
        plug(traj_gen.q,                  ros.traj_gen_q_ros);
        plug(traj_gen.dq,                 ros.traj_gen_dq_ros);
        plug(traj_gen.ddq,                ros.traj_gen_ddq_ros);
        plug(traj_gen.fRightFoot,         ros.traj_gen_fRightFoot_ros);
    if(ctrl_manager!=None):
        ros.add('vector', 'ctrl_manager_pwmDes_ros',                'ctrl_manager_pwmDes');
        ros.add('vector', 'ctrl_manager_pwmDesSafe_ros',                'ctrl_manager_pwmDesSafe');
        ros.add('vector', 'ctrl_manager_signOfControlFiltered_ros', 'ctrl_manager_signOfControlFiltered');
        ros.add('vector', 'ctrl_manager_signOfControl_ros',         'ctrl_manager_signOfControl');
        plug(ctrl_manager.pwmDes,                   ros.ctrl_manager_pwmDes_ros);
        plug(ctrl_manager.pwmDesSafe,                   ros.ctrl_manager_pwmDesSafe_ros);
        plug(ctrl_manager.signOfControlFiltered,    ros.ctrl_manager_signOfControlFiltered_ros);
        plug(ctrl_manager.signOfControl,            ros.ctrl_manager_signOfControl_ros);
    if(inv_dyn!=None):
        ros.add('vector', 'inv_dyn_tauDes_ros',    'inv_dyn_tauDes');
        ros.add('vector', 'inv_dyn_tauFF_ros',     'inv_dyn_tauFF');
        ros.add('vector', 'inv_dyn_tauFB_ros',     'inv_dyn_tauFB');
        ros.add('vector', 'inv_dyn_ddqDes_ros',    'inv_dyn_ddqDes');
        ros.add('vector', 'inv_dyn_qError_ros',    'inv_dyn_qError');
        plug(inv_dyn.tauDes,    ros.inv_dyn_tauDes_ros);
        plug(inv_dyn.tauFF,     ros.inv_dyn_tauFF_ros);
        plug(inv_dyn.tauFB,     ros.inv_dyn_tauFB_ros);
        plug(inv_dyn.ddqDes,    ros.inv_dyn_ddqDes_ros);
        plug(inv_dyn.qError,    ros.inv_dyn_qError_ros);
    if(adm_ctrl!=None):
        ros.add('vector', 'adm_ctrl_qDes_ros',              'adm_ctrl_qDes');
        ros.add('vector', 'adm_ctrl_dqDes_ros',             'adm_ctrl_dqDes');
        ros.add('vector', 'adm_ctrl_fRightFootError_ros',   'adm_ctrl_fRightFootError');
        plug(adm_ctrl.qDes,             ros.adm_ctrl_qDes_ros);
        plug(adm_ctrl.dqDes,            ros.adm_ctrl_dqDes_ros);
        plug(adm_ctrl.fRightFootError,  ros.adm_ctrl_fRightFootError_ros);
    if(ff_locator!=None):
        plug(ffLocator.base6dFromFoot_encoders,    ros.robotState_ros);
    if(floatingBase!=None):
        ros.add('vector', 'floatingBase_pos_ros', 'floatingBase_pos');
        plug(floatingBase.soutPos, ros.floatingBase_pos_ros);
    
    
    
    return ros;
    
    
def addTrace(tracer, entity, signalName):
    """
    Add a signal to a tracer
    """
    signal = '{0}.{1}'.format(entity.name, signalName);
    filename = '{0}-{1}'.format(entity.name, signalName);
    tracer.add(signal, filename);
    
def addSignalsToTracer(tracer, device, traj_gen, torque_ctrl):
    addTrace(tracer,device,'robotState');
    addTrace(tracer,device,'gyrometer');
    addTrace(tracer,device,'accelerometer');
    addTrace(tracer,device,'forceRLEG');
    addTrace(tracer,device,'forceLLEG');
    addTrace(tracer,device,'forceRARM');
    addTrace(tracer,device,'forceLARM');
    addTrace(tracer,device,'control');
    addTrace(tracer,device,'currents');
#    addTrace(tracer,device,'ptorque');
#    addTrace(tracer,device,'p_gains');
#    addTrace(tracer,traj_gen,'q');
#    addTrace(tracer,traj_gen,'dq');
#    addTrace(tracer,traj_gen,'ddq');
#    addTrace(tracer,traj_gen,'fRightFoot');
#    addTrace(tracer,estimator,'jointsVelocities');
#    addTrace(tracer,estimator,'jointsAccelerations');
#    addTrace(tracer,estimator,'jointsTorques');
#    addTrace(tracer,estimator,'contactWrenchRightFoot');
#    addTrace(tracer,inv_dyn,'tauDes');
#    addTrace(tracer,inv_dyn,'ddqDes');
#    addTrace(tracer,torque_ctrl,'jointsPositionsDesired');
#    addTrace(tracer,torque_ctrl,'deltaQ_ff');
#    addTrace(tracer,torque_ctrl,'deltaQ_fb');
#    addTrace(tracer,torque_ctrl,'deltaQ_friction');


def create_tracer(device, traj_gen=None, estimator=None, inv_dyn=None, torque_ctrl=None):
    tracer = TracerRealTime('motor_id_trace');
    tracer.setBufferSize(80*(2**20));
    tracer.open('/tmp/','dg_','.dat');
    device.after.addSignal('{0}.triger'.format(tracer.name));

    addSignalsToTracer(tracer, device, traj_gen, torque_ctrl);
        
    with open('/tmp/dg_info.dat', 'a') as f:
        if(estimator!=None):
            f.write('Estimator encoder delay: {0}\n'.format(estimator.getDelayEnc()));
            f.write('Estimator F/T sensors delay: {0}\n'.format(estimator.getDelayFTsens()));
            f.write('Estimator accelerometer delay: {0}\n'.format(estimator.getDelayAcc()));
            f.write('Estimator gyroscope delay: {0}\n'.format(estimator.getDelayGyro()));
            f.write('Estimator use reference velocities: {0}\n'.format(estimator.getUseRefJointVel()));
            f.write('Estimator use reference accelerations: {0}\n'.format(estimator.getUseRefJointAcc()));
            f.write('Estimator use raw encoders: {0}\n'.format(estimator.getUseRawEncoders()));
            f.write('Estimator use f/t sensors: {0}\n'.format(estimator.getUseFTsensors()));
            f.write('Estimator f/t sensor offsets: {0}\n'.format(estimator.getFTsensorOffsets()));
        if(inv_dyn!=None):
            f.write('Inv dyn Ks: {0}\n'.format(inv_dyn.Kp.value));
            f.write('Inv dyn Kd: {0}\n'.format(inv_dyn.Kd.value));
            f.write('Inv dyn Kf: {0}\n'.format(inv_dyn.Kf.value));
            f.write('Inv dyn Ki: {0}\n'.format(inv_dyn.Ki.value));
        if(torque_ctrl!=None):
            f.write('Torque ctrl KpTorque: {0}\n'.format (torque_ctrl.KpTorque.value ));
            f.write('Torque ctrl KpCurrent: {0}\n'.format(torque_ctrl.KpCurrent.value));
            f.write('Torque ctrl K_tau: {0}\n'.format(torque_ctrl.k_tau.value));
            f.write('Torque ctrl K_v: {0}\n'.format(torque_ctrl.k_v.value));
    f.close();
    return tracer;

def reset_tracer(device,tracer):
    tracer.stop();
    tracer.dump();
    tracer.close();
    tracer.clear();
    tracer = create_tracer(device);
    return tracer;
    


