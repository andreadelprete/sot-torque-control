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
kp_pos[0] = 100;
# PARAMETERS OF R_hip_r JOINT 1
kp_pos[1] = 100;
# PARAMETERS OF R_hip_p JOINT 2
kp_pos[2] = 100; 
# PARAMETERS OF R_knee JOINT 3
kp_pos[3] = 100;
# PARAMETERS OF R_ankle pitch JOINT 4
kp_pos[4] = 50;
# PARAMETERS OF R_ankle roll JOINT 5
kp_pos[5] = 40;


# PARAMETERS OF L_hip_y JOINT 6
kp_pos[6] = 100;
# PARAMETERS OF L_hip_r JOINT 7
kp_pos[7] = 100;
# PARAMETERS OF L_hip_p JOINT 8
kp_pos[8] = 100; 
# PARAMETERS OF L_knee JOINT 9
kp_pos[9] = 100;
# PARAMETERS OF L_ankle pitch JOINT 10
kp_pos[10] = 50;
# PARAMETERS OF L_ankle roll JOINT 11
kp_pos[11] = 40;

# PARAMTERS of torso yaw and pitch (12, 13)
kp_pos[12] = 150;
kp_pos[13] = 150;

# PARAMTERS of head yaw and pitch (14, 15)
kp_pos[14] = 30;
kp_pos[15] = 30;

# PARAMETERS OF right shoulder pitch JOINT 16
kp_pos[16] = 150;
# PARAMETERS OF R_shoulder roll JOINT 17
kp_pos[17] = 100; # 
# PARAMETERS OF R_shoulder yaw JOINT 18
kp_pos[18] = 150; # 
# PARAMETERS OF right elbow joint 19
kp_pos[19] = 150
# PARAMETERS OF right wrist yaw joint 20
kp_pos[20] = 150
# PARAMETERS OF right wrist pitch joint 21
kp_pos[21] = 150
# PARAMETERS OF right hand joint 22
kp_pos[22] = 20


# PARAMETERS OF left shoulder pitch JOINT 23
kp_pos[23] = 150;
# PARAMETERS OF L_shoulder roll JOINT 24
kp_pos[24] = 100; # 
# PARAMETERS OF L_shoulder yaw JOINT 25
kp_pos[25] = 150; # 
# PARAMETERS OF left elbow joint 26
kp_pos[26] = 150
# PARAMETERS OF left wrist yaw joint 27
kp_pos[27] = 150
# PARAMETERS OF left wrist pitch joint 28
kp_pos[28] = 150
# PARAMETERS OF left hand joint 29
kp_pos[29] = 20

