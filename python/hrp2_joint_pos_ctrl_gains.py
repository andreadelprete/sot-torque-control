# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
from numpy import zeros as zeros

NJ = 30;
kp_pos = zeros(NJ);   # joint position control proportional gains
kd_pos = zeros(NJ);   # joint position control derivative gains
ki_pos = zeros(NJ);   # joint position control integral gains
pwm_max = zeros(NJ);


# PARAMETERS OF R_hip_y JOINT 0
kp_pos[0] = 800;
# PARAMETERS OF R_hip_r JOINT 1
kp_pos[1] = 800;
# PARAMETERS OF R_hip_p JOINT 2
kp_pos[2] = 800; 
# PARAMETERS OF R_knee JOINT 3
kp_pos[3] = 800;
# PARAMETERS OF R_ankle pitch JOINT 4
kp_pos[4] = 800;
# PARAMETERS OF R_ankle roll JOINT 5
kp_pos[5] = 800;


# PARAMETERS OF L_hip_y JOINT 6
kp_pos[6] = 800;
# PARAMETERS OF L_hip_r JOINT 7
kp_pos[7] = 800;
# PARAMETERS OF L_hip_p JOINT 8
kp_pos[8] = 800; 
# PARAMETERS OF L_knee JOINT 9
kp_pos[9] = 800;
# PARAMETERS OF L_ankle pitch JOINT 10
kp_pos[10] = 800;
# PARAMETERS OF L_ankle roll JOINT 11
kp_pos[11] = 800;

# PARAMTERS of torso yaw and pitch (12, 13)
kp_pos[12] = 800;
kp_pos[13] = 800;

# PARAMTERS of head yaw and pitch (14, 15)
kp_pos[14] = 50;
kp_pos[15] = 50;

# PARAMETERS OF right shoulder pitch JOINT 16
kp_pos[16] = 500;
# PARAMETERS OF R_shoulder roll JOINT 17
kp_pos[17] = 500; # 
# PARAMETERS OF R_shoulder yaw JOINT 18
kp_pos[18] = 500; # 
# PARAMETERS OF right elbow joint 19
kp_pos[19] = 500
# PARAMETERS OF right wrist yaw joint 20
kp_pos[20] = 500
# PARAMETERS OF right wrist pitch joint 21
kp_pos[21] = 500
# PARAMETERS OF right hand joint 22
kp_pos[22] = 50


# PARAMETERS OF left shoulder pitch JOINT 23
kp_pos[23] = 500;
# PARAMETERS OF L_shoulder roll JOINT 24
kp_pos[24] = 500; # 
# PARAMETERS OF L_shoulder yaw JOINT 25
kp_pos[25] = 500; # 
# PARAMETERS OF left elbow joint 26
kp_pos[26] = 500
# PARAMETERS OF left wrist yaw joint 27
kp_pos[27] = 500
# PARAMETERS OF left wrist pitch joint 28
kp_pos[28] = 500
# PARAMETERS OF left hand joint 29
kp_pos[29] = 50

