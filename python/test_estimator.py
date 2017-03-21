# -*- coding: utf-8 -*-1
"""
2014, LAAS/CNRS
@author: Andrea Del Prete
"""

from dynamic_graph import plug
from create_entities_utils import *
from hrp2_motors_parameters import *
import numpy as np
from time import sleep

''' Main function to call before starting the graph. '''
def main_pre_start_pwm(robot,dt=0.001,delay=0.01):
    robot.device.setControlInputType('position');
    traj_gen        = create_trajectory_generator(robot.device, dt);
    estimator       = create_estimator(robot.device, dt, delay, traj_gen);
    pos_ctrl        = create_position_controller(robot.device, estimator, dt, traj_gen);
    torque_ctrl     = create_torque_controller(robot.device, estimator);    
    inv_dyn         = create_inverse_dynamics(robot.device, estimator, torque_ctrl, traj_gen, dt);    
    ctrl_manager    = create_ctrl_manager(robot.device, torque_ctrl, pos_ctrl, inv_dyn, estimator, dt);
    ff_locator      = create_free_flyer_locator(robot,'/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14.urdf')
    flex_est        = create_flex_estimator(robot,dt)
    floatingBase    = create_floatingBase(flex_est,ff_locator)

    estimator.gyroscope.value = (0.0, 0.0, 0.0);
    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    return (estimator,torque_ctrl,traj_gen,ctrl_manager,inv_dyn,pos_ctrl,ff_locator,flex_est,floatingBase);

''' Main function to call before starting the graph. '''
def main_pre_start_pwm_noTorqueControl(robot,dt=0.001,delay=0.01):
    robot.device.setControlInputType('position');
    traj_gen        = create_trajectory_generator(robot.device, dt);
    estimator       = create_estimator(robot.device, dt, delay, traj_gen);
    pos_ctrl        = create_position_controller(robot.device, estimator, dt, traj_gen);
    torque_ctrl     = create_torque_controller(robot.device, estimator);    
    #inv_dyn         = create_inverse_dynamics(robot.device, estimator, torque_ctrl, traj_gen, dt);    
    ctrl_manager    = create_ctrl_manager_noTorqueControl (robot.device, torque_ctrl, pos_ctrl, estimator, dt);
        
    estimator.gyroscope.value = (0.0, 0.0, 0.0);
    estimator.accelerometer.value = (0.0, 0.0, 9.81);
    return (estimator,torque_ctrl,traj_gen,ctrl_manager,pos_ctrl);




''' Main function to call before starting the graph. '''
def main_pre_start(task,robot,dt=0.001,delay=0.01):
    robot.device.setControlInputType('position');
    traj_gen        = create_trajectory_generator(robot.device, dt);
    estimator       = create_estimator(robot.device, dt, delay, traj_gen);
    torque_ctrl     = create_torque_controller(robot.device, estimator);    
    inv_dyn         = create_inverse_dynamics(robot.device, estimator, torque_ctrl, traj_gen, dt);    
    ctrl_manager    = create_ctrl_manager(robot.device, torque_ctrl, traj_gen, inv_dyn, estimator, dt);
    adm_ctrl        = create_admittance_ctrl(robot.device, estimator, ctrl_manager, traj_gen, dt);
    
    if(task=='identification'):
        estimator.setUseRawEncoders(False);
        estimator.setUseRefJointVel(False);
        estimator.setUseRefJointAcc(False);
#        estimator.setUseFTsensors(False);
        robot.device.after.addSignal('estimator.jointsVelocities');
    elif(task=='torque_ctrl'):
        ctrl_manager.setCtrlMode('rhy','torque');
        ctrl_manager.setCtrlMode('rhr','torque');
        ctrl_manager.setCtrlMode('rhp','torque');
        ctrl_manager.setCtrlMode('rk','torque');
        ctrl_manager.setCtrlMode('rap','torque');
        ctrl_manager.setCtrlMode('rar','torque');
    
    estimator.gyroscope.value = (0.0, 0.0, 0.0);
    #estimator.accelerometer.value = (0.0, 0.0, 9.81);
        
    return (estimator,torque_ctrl,traj_gen,ctrl_manager,inv_dyn,adm_ctrl);

    
''' Main function to call after having started the graph. '''
def main_post_start(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl, ff_locator, floatingBase):
    ros = create_ros_topics(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl, ff_locator, floatingBase);
    return ros;
    
def start_tracer(robot, estimator, torque_ctrl, traj_gen, ctrl_manager, inv_dyn, adm_ctrl):
    tracer = create_tracer(robot.device, traj_gen, estimator, inv_dyn, torque_ctrl);
    tracer.start();
    return tracer;

def start_admittance_ctrl_RL(ctrl_manager):
    ctrl_manager.setCtrlMode('rhy','adm');
    ctrl_manager.setCtrlMode('rhr','adm');
    ctrl_manager.setCtrlMode('rhp','adm');
    ctrl_manager.setCtrlMode('rk','adm');
    ctrl_manager.setCtrlMode('rap','adm');
    ctrl_manager.setCtrlMode('rar','adm');

def move_to_initial_configuration(traj_gen):
    traj_gen.moveJoint('rhy', 0, 4.0);
    traj_gen.moveJoint('rhr', 0, 4.0);
    traj_gen.moveJoint('rhp',-0.6, 4);
    traj_gen.moveJoint('rk', 1.1, 4);
    traj_gen.moveJoint('rap',-0.6, 4);    
    traj_gen.moveJoint('rar', 0, 4.0);
    traj_gen.moveJoint('lhy', 0, 4.0);
    traj_gen.moveJoint('lhp', 0, 4.0);
    traj_gen.moveJoint('lhr', 0.5, 4.0);
    traj_gen.moveJoint('lk', 1.7, 4.0);
    traj_gen.moveJoint('lap', 0, 4.0);
    traj_gen.moveJoint('lar', 0, 4.0);
    
def smoothly_set_signal_to_zero(sig):
    v = np.array(sig.value);
    for i in range(40):
        v = 0.95*v;
        sig.value = tuple(v);
        sleep(1);
    print 'Setting signal to zero';
    v[:] = 0.0;
    sig.value = tuple(v);
    
def monitor_tracking_error(sig, sigRef, dt, time):
    N = int(time/dt);
    err = np.zeros((N,6));
    for i in range(N):
        err[i,:] = np.array(sig.value) - np.array(sigRef.value);
        sleep(dt);
    for i in range(6):
        print 'Max tracking error for axis %d:         %.2f' % (i, np.max(np.abs(err[:,i])));
        print 'Mean square tracking error for axis %d: %.2f' % (i, np.linalg.norm(err[:,i])/N);
    
def dump_signal_to_file(sig_list, index, filename, T, dt):
    N = int(T/dt);
    m = len(sig_list);
    f= open('/tmp/'+filename, 'a', 1);
    for t in range(N):
        for s in sig_list:
            f.write('{0}\t'.format(s.value[index]))
        f.write('\n');
        sleep(dt);
    f.close();
    
