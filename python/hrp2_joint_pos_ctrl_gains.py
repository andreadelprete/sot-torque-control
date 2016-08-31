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

# PARAMETERS OF left elbow JOINT 26
kp_pos[26] = 5000;
# PARAMETERS OF R_hip_y JOINT 0
kp_pos[0] = 10000;
kd_pos[0] = 0;
# PARAMETERS OF R_hip_r JOINT 1
kp_pos[1] = 10000; #
kd_pos[1] = 0;
# PARAMETERS OF R_hip_p JOINT 2
kp_pos[2] = 10000; 
kd_pos[2] = 0;
# PARAMETERS OF R_knee JOINT 3
kp_pos[3] = 10000;  # 
kd_pos[3] = 0;
# PARAMETERS OF R_ankle pitch JOINT 4
kp_pos[4] = 5000;  # 
kd_pos[4] = 0;
# PARAMETERS OF R_ankle roll JOINT 5
kp_pos[5] = 0; # 
kd_pos[5] = 0; # 

# PARAMTERS of torso yaw and pitch (12, 13)
kp_pos[12] = 15000;
kp_pos[13] = 15000;

# PARAMETERS OF right shoulder pitch JOINT 16
kp_pos[16] = 15000;
# PARAMETERS OF R_shoulder roll JOINT 17
kp_pos[17] = 10000; # 
# PARAMETERS OF R_shoulder yaw JOINT 18
kp_pos[18] = 15000; # 
# PARAMETERS OF right elbow joint 19
kp_pos[19] = 15000
# PARAMETERS OF right wrist yaw joint 20
kp_pos[20] = 15000
# PARAMETERS OF right wrist pitch joint 21
kp_pos[21] = 15000

