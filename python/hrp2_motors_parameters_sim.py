# -*- coding: utf-8 -*-
"""
Created on Mon Feb  9 13:55:16 2015

@author: adelpret
"""
from numpy import zeros as zeros
import numpy as np

NJ = 30;
k_p = zeros(NJ)
k_tau = zeros(NJ);
k_v   = zeros(NJ);
f_k1p = zeros(NJ);
f_k2p = zeros(NJ);
f_k3p = zeros(NJ);
f_k1n = zeros(NJ);
f_k2n = zeros(NJ);
f_k3n = zeros(NJ);
f_q1p = zeros(NJ);
f_q2p = zeros(NJ);
f_q3p = zeros(NJ);
f_q1n = zeros(NJ);
f_q2n = zeros(NJ);
f_q3n = zeros(NJ);
f_tau1p = zeros(NJ);
f_tau2p = zeros(NJ);
f_tau1n = zeros(NJ);
f_tau2n = zeros(NJ);
g_k1p = zeros(NJ);
g_k2p = zeros(NJ);
g_k3p = zeros(NJ);
g_k1n = zeros(NJ);
g_k2n = zeros(NJ);
g_k3n = zeros(NJ);
g_q1p = zeros(NJ);
g_q2p = zeros(NJ);
g_q3p = zeros(NJ);
g_q1n = zeros(NJ);
g_q2n = zeros(NJ);
g_q3n = zeros(NJ);
g_dq1p = zeros(NJ);
g_dq2p = zeros(NJ);
g_dq1n = zeros(NJ);
g_dq2n = zeros(NJ);

# 10,333,241,10,3,3,

# PARAMETERS OF R_hip_y JOINT 0
k_tau[0] = 1.0/10.0;
# PARAMETERS OF R_hip_r JOINT 1
k_tau[1] = 1.0/1000.0
# PARAMETERS OF R_hip_p JOINT 2
k_tau[2] = 1.0/1000.0
# PARAMETERS OF R_knee JOINT 3
k_tau[3] = 0.1
# PARAMETERS OF R_ankle pitch JOINT 4
k_tau[4] = 1.0/3.0
# PARAMETERS OF R_ankle roll JOINT 5
k_tau[5] = 1.0/3.0
# PARAMETERS OF Left leg
k_tau[6:12] = k_tau[0:6]
# TORSO
k_tau[12] = 1.0/50.0
k_tau[13] = 1.0/50.0
# HEAD
k_tau[14] = 1.0/1.0
k_tau[15] = 1.0/1.0
# RIGHT ARM
k_tau[16:22] = 1.0/1.0
k_tau[22] = 1.0/0.1;
# LEFT ARM
k_tau[23:29] = 1.0/1.0
k_tau[29] = 1.0/0.1;

k_v = 2*k_tau*np.sqrt(1.0/k_tau);

# PARAMETERS OF 1_(R_hip_y) JOINT 0
f_k1p[0] = 0.000388
f_k2p[0] = 0.002431
f_k3p[0] = 0.000273
f_k1n[0] = 0.000275
f_k2n[0] = 0.002916
f_k3n[0] = 0.000422
f_q1p[0] = -0.003380
f_q2p[0] = -0.012854
f_q3p[0] = 0.002403
f_q1n[0] = -0.002494
f_q2n[0] = 0.011919
f_q3n[0] = 0.003306
f_tau1p[0] = 4.647462
f_tau2p[0] = 7.053441
f_tau1n[0] = -5.447130
f_tau2n[0] = -3.455461
# PARAMETERS OF 2_(R_hip_r) JOINT 1
f_k1p[1] = 0.000040
f_k2p[1] = 0.000833
f_k3p[1] = 0.000028
f_k1n[1] = 0.000038
f_k2n[1] = 0.000833
f_k3n[1] = 0.000029
f_q1p[1] = -0.000406
f_q2p[1] = -0.004123
f_q3p[1] = 0.000255
f_q1n[1] = -0.000237
f_q2n[1] = 0.002944
f_q3n[1] = 0.000373
f_tau1p[1] = 4.680849
f_tau2p[1] = 5.434831
f_tau1n[1] = -3.987784
f_tau2n[1] = -3.202386
# PARAMETERS OF 3_(R_hip_p) JOINT 2
f_k1p[2] = 0.000156
f_k2p[2] = 0.000833
f_k3p[2] = 0.000080
f_k1n[2] = 0.000089
f_k2n[2] = 0.000833
f_k3n[2] = 0.000110
f_q1p[2] = -0.000700
f_q2p[2] = -0.002376
f_q3p[2] = 0.000759
f_q1n[2] = -0.000774
f_q2n[2] = 0.007945
f_q3n[2] = 0.001718
f_tau1p[2] = 2.508688
f_tau2p[2] = 4.164688
f_tau1n[2] = -11.694394
f_tau2n[2] = -8.598394
# PARAMETERS OF 4_(R_knee_p) JOINT 3
f_k1p[3] = 0.000064
f_k2p[3] = 0.002415
f_k3p[3] = 0.000042
f_k1n[3] = 0.000051
f_k2n[3] = 0.000833
f_k3n[3] = 0.000048
f_q1p[3] = -0.000342
f_q2p[3] = -0.006247
f_q3p[3] = 0.000448
f_q1n[3] = -0.000147
f_q2n[3] = 0.006358
f_q3n[3] = 0.000734
f_tau1p[3] = 2.521347
f_tau2p[3] = 2.827010
f_tau1n[3] = -8.327572
f_tau2n[3] = -7.158899
# PARAMETERS OF 5_(R_ankle_p) JOINT 4
f_k1p[4] = 0.000200
f_k2p[4] = 0.001963
f_k3p[4] = 0.000129
f_k1n[4] = 0.000166
f_k2n[4] = 0.005242
f_k3n[4] = 0.000165
f_q1p[4] = -0.001125
f_q2p[4] = -0.004472
f_q3p[4] = 0.001398
f_q1n[4] = -0.000370
f_q2n[4] = 0.020556
f_q3n[4] = 0.001931
f_tau1p[4] = 1.895690
f_tau2p[4] = 3.193865
f_tau1n[4] = -4.124708
f_tau2n[4] = -3.671868
# PARAMETERS OF 6_(R_ankle_r) JOINT 5
f_k1p[5] = 0.000230
f_k2p[5] = 0.002760
f_k3p[5] = 0.000210
f_k1n[5] = 0.000218
f_k2n[5] = 0.001577
f_k3n[5] = 0.000209
f_q1p[5] = -0.000852
f_q2p[5] = -0.005957
f_q3p[5] = 0.000768
f_q1n[5] = -0.000405
f_q2n[5] = 0.003929
f_q3n[5] = 0.001234
f_tau1p[5] = 2.010004
f_tau2p[5] = 2.647346
f_tau1n[5] = -3.188489
f_tau2n[5] = -1.953347
# PARAMETERS OF 7_(L_hip_y) JOINT 6
f_k1p[6] = 0.000401
f_k2p[6] = 100.000000
f_k3p[6] = 0.000294
f_k1n[6] = 0.000271
f_k2n[6] = 0.001836
f_k3n[6] = 0.000415
f_q1p[6] = -0.003615
f_q2p[6] = -398.589186
f_q3p[6] = 0.002126
f_q1n[6] = -0.001888
f_q2n[6] = 0.009343
f_q3n[6] = 0.003790
f_tau1p[6] = 3.985872
f_tau2p[6] = 3.985925
f_tau1n[6] = -7.161778
f_tau2n[6] = -3.908519
# PARAMETERS OF 8_(L_hip_r) JOINT 7
f_k1p[7] = 0.000038
f_k2p[7] = 0.000864
f_k3p[7] = 0.000038
f_k1n[7] = 0.000034
f_k2n[7] = 0.000833
f_k3n[7] = 0.000045
f_q1p[7] = -0.000346
f_q2p[7] = -0.001599
f_q3p[7] = 0.000345
f_q1n[7] = -0.000284
f_q2n[7] = 0.003013
f_q3n[7] = 0.000438
f_tau1p[7] = 1.502178
f_tau2p[7] = 2.350674
f_tau1n[7] = -4.133966
f_tau2n[7] = -3.254320
# PARAMETERS OF 9_(L_hip_p) JOINT 8
f_k1p[8] = 0.000116
f_k2p[8] = 0.000833
f_k3p[8] = 0.000086
f_k1n[8] = 0.000092
f_k2n[8] = 0.000833
f_k3n[8] = 0.000124
f_q1p[8] = -0.002292
f_q2p[8] = -0.011197
f_q3p[8] = 0.000472
f_q1n[8] = -0.001561
f_q2n[8] = -0.002991
f_q3n[8] = 0.000369
f_tau1p[8] = 12.411460
f_tau2p[8] = 15.594460
f_tau1n[8] = 1.935234
f_tau2n[8] = 4.742034
# PARAMETERS OF 10_(L_knee_p) JOINT 9
f_k1p[9] = 0.000045
f_k2p[9] = 0.000833
f_k3p[9] = 0.000047
f_k1n[9] = 0.000043
f_k2n[9] = 0.000833
f_k3n[9] = 0.000055
f_q1p[9] = -0.000865
f_q2p[9] = -0.006720
f_q3p[9] = 0.000149
f_q1n[9] = -0.000611
f_q2n[9] = 0.000404
f_q3n[9] = 0.000247
f_tau1p[9] = 7.420875
f_tau2p[9] = 8.740275
f_tau1n[9] = -1.278539
f_tau2n[9] = -0.185339
# PARAMETERS OF 11_(L_ankle_p) JOINT 10
f_k1p[10] = 0.000175
f_k2p[10] = 0.001507
f_k3p[10] = 0.000156
f_k1n[10] = 0.000136
f_k2n[10] = 0.002136
f_k3n[10] = 0.000165
f_q1p[10] = -0.001886
f_q2p[10] = -0.006298
f_q3p[10] = 0.000125
f_q1n[10] = -0.001367
f_q2n[10] = 0.003247
f_q3n[10] = 0.000809
f_tau1p[10] = 3.311642
f_tau2p[10] = 4.748123
f_tau1n[10] = -2.312898
f_tau2n[10] = -1.234218
# PARAMETERS OF 12_(L_ankle_r) JOINT 11
f_k1p[11] = 0.000234
f_k2p[11] = 0.003025
f_k3p[11] = 0.000228
f_k1n[11] = 0.000215
f_k2n[11] = 0.002221
f_k3n[11] = 0.000237
f_q1p[11] = -0.001079
f_q2p[11] = -0.004896
f_q3p[11] = 0.000599
f_q1n[11] = -0.000865
f_q2n[11] = 0.003553
f_q3n[11] = 0.000927
f_tau1p[11] = 1.358952
f_tau2p[11] = 1.961382
f_tau1n[11] = -2.207792
f_tau2n[11] = -1.330901

# PARAMETERS OF 3\_(R\_hip\_p) JOINT 2
g_k1p[2] = 0.012274
g_k2p[2] = 0.025269
g_k3p[2] = 0.012301
g_k1n[2] = 0.012188
g_k2n[2] = 0.032493
g_k3n[2] = 0.012035
g_q1p[2] = -0.003859
g_q2p[2] = -0.003162
g_q3p[2] = -0.000673
g_q1n[2] = -0.001845
g_q2n[2] = -0.000563
g_q3n[2] = 0.001133
g_dq1p[2] = -0.053629
g_dq2p[2] = 0.191807
g_dq1n[2] = -0.063182
g_dq2n[2] = 0.082912
