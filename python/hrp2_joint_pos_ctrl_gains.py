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
kp_pos[0] = 10000;
kd_pos[0] = 0;
# PARAMETERS OF R_hip_r JOINT 1
kp_pos[1] = 10000;
kd_pos[1] = 0;
# PARAMETERS OF R_hip_p JOINT 2
kp_pos[2] = 10000; 
kd_pos[2] = 0;
# PARAMETERS OF R_knee JOINT 3
kp_pos[3] = 10000;
kd_pos[3] = 0;
# PARAMETERS OF R_ankle pitch JOINT 4
kp_pos[4] = 5000;
kd_pos[4] = 0;
# PARAMETERS OF R_ankle roll JOINT 5
kp_pos[5] = 4000;
kd_pos[5] = 0; 


# PARAMETERS OF L_hip_y JOINT 6
kp_pos[6] = 10000;
kd_pos[6] = 0;
# PARAMETERS OF L_hip_r JOINT 7
kp_pos[7] = 10000;
kd_pos[7] = 0;
# PARAMETERS OF L_hip_p JOINT 8
kp_pos[8] = 10000; 
kd_pos[8] = 0;
# PARAMETERS OF L_knee JOINT 9
kp_pos[9] = 10000;
kd_pos[9] = 0;
# PARAMETERS OF L_ankle pitch JOINT 10
kp_pos[10] = 5000;
kd_pos[10] = 0;
# PARAMETERS OF L_ankle roll JOINT 11
kp_pos[11] = 4000;
kd_pos[11] = 0;

# PARAMTERS of torso yaw and pitch (12, 13)
kp_pos[12] = 15000;
kp_pos[13] = 15000;

# PARAMTERS of head yaw and pitch (14, 15)
kp_pos[14] = 3000;
kp_pos[15] = 3000;

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
# PARAMETERS OF right hand joint 22
kp_pos[22] = 2000


# PARAMETERS OF left shoulder pitch JOINT 23
kp_pos[23] = 15000;
# PARAMETERS OF L_shoulder roll JOINT 24
kp_pos[24] = 10000; # 
# PARAMETERS OF L_shoulder yaw JOINT 25
kp_pos[25] = 15000; # 
# PARAMETERS OF left elbow joint 26
kp_pos[26] = 15000
# PARAMETERS OF left wrist yaw joint 27
kp_pos[27] = 15000
# PARAMETERS OF left wrist pitch joint 28
kp_pos[28] = 15000
# PARAMETERS OF left hand joint 29
kp_pos[29] = 2000

