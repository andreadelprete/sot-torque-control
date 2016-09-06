# -*- coding: utf-8 -*-
from scipy import signal
from scipy import ndimage
import numpy as np
import sys
from IPython import embed
from plot_utils import plot3d
from plot_utils import plot_x_vs_y
from plot_utils import saveCurrentFigure
import plot_utils
import matplotlib.pyplot as plt
from motor_model import Motor_model
'''
motor model :

Kt*i(t) = J*a(t) + Kd * v(t) + Tauf + Tau
i(t) = J/Kt*a(t) + Kd/Kt * v(t) + Tauf/Kt + Tau/Kt
Tau = Kt*i(t) - J*a(t) - Kd * v(t) - Tauf
'''

tauf_pos = 2.007070
tauf_neg = -1.520840

#~ tauf_pos = 7
#~ tauf_neg = -7.0
Kt=1.346776
Kv_pos = 0.937689
Kv_neg = 0.514448
#~ i_f_pos = 1.249641
#~ i_f_neg = -1.265535 
JoverKt_pos = 0.204553
JoverKt_neg = 0.183804

J = (JoverKt_pos*Kt+JoverKt_neg*Kt) * 0.5

motor = Motor_model(tauf_pos, tauf_neg, Kt, Kv_pos, Kv_neg, J)
#~ motor.display()
# Test smooth tauf
#~ dqs=[]
#~ taufs=[]
#~ for dq in np.linspace(-1,1,1000):
    #~ dqs.append(dq)
    #~ taufs.append(motor.get_smooth_tauf(dq))
#~ plt.plot(dqs,taufs)
#~ plt.show()


''' Solve the least square problem:
    minimize   || A*x-b ||^2
'''
def solveLeastSquare(A, b):
    return np.linalg.pinv(A)*np.matrix(b).T;

DATA_SET  = 1;
FOLDER_ID = 4;
DATA_FILE_NAME = 'data.npz';

ZERO_VELOCITY_THRESHOLD     = 0.01
ZERO_ACCELERATION_THRESHOLD = 1.0
ZERO_JERK_THRESHOLD         = 2.0
SHOW_THRESHOLD_EFFECT = True


if(DATA_SET==1):
    if(FOLDER_ID==1):
        data_folder = '../../results/20160712_170026_rsp_torque_id/';
        JOINT_ID = 16;
    elif(FOLDER_ID==2):
        data_folder = '../../results/20160712_171735_rsp_const_vel/';
        JOINT_ID = 16;
    elif(FOLDER_ID==3):
        data_folder = '../../results/20160712_182523_rsp_const_acc/';
        JOINT_ID = 16;
    elif(FOLDER_ID==4):
        data_folder = '../../results/20160720_134957_rsp_friction_id_ext/';
        #~ data_folder = '../../results/20160720_143905_rsp_torque_id_noGravity/';
        #~ data_folder = '../../results/20160712_171735_rsp_const_vel/'
        #~ data_folder = '../../results/20160712_182523_rsp_const_acc/';
        JOINT_ID = 16;
    else:
        print 'ERROR: UNKNOWN FOLDER_ID';
if(DATA_SET==2):
    if(FOLDER_ID==1):
        #~ data_folder = '../../results/20160720_132041_rsp_torque_id/';
        #~ data_folder = '../../results/20160720_132429_rsp_torque_id/';
        data_folder = '../../results/20160720_143905_rsp_torque_id_noGravity/';
        JOINT_ID = 16;
    elif(FOLDER_ID==2):
        #~ data_folder = '../../results/20160720_134957_rsp_friction_id_ext/';
        data_folder = '../../results/20160722_144631_rsp_const_vel/';
        JOINT_ID = 16;
    elif(FOLDER_ID==3):
        data_folder = '../../rien/';
        JOINT_ID = 16;
    else:
        print 'ERROR: UNKNOWN FOLDER_ID';
DATA_FILE_NAME = 'data_j'+str(JOINT_ID)+'.npz';

#~ 
#~ PLOT_LOW_VEL_DATA_3D    = True;
#~ PLOT_LOW_VEL_DATA       = True;
#~ PLOT_RESIDUALS          = False;
#~ PLOT_3D_SURFACE         = True;
#~ PLOT_FRICTION_DATA      = False;
#~ PLOT_PIECE_WISE_LINEAR  = True;
#~ PLOT_ANIMATED_DATA      = False;
#~ COMPARE_TWO_DATASETS    = False;
#~ SHOW_PLOTS              = True;

plot_utils.FIGURE_PATH      = data_folder;
plot_utils.SAVE_FIGURES     = False;
plot_utils.SHOW_FIGURES     = True;
plot_utils.SHOW_LEGENDS     = True;
plot_utils.LINE_ALPHA       = 0.7;

''' Load data from file '''
try:
    data = np.load(data_folder+DATA_FILE_NAME);
    if(len(data['enc'].shape)==1):
        enc = np.squeeze(data['enc']);
        dq = np.squeeze(data['dq']);
        tau = np.squeeze(data['tau']);
        ctrl = np.squeeze(data['ctrl']);
        current = np.squeeze(data['current']);
    else:
        enc = np.squeeze(data['enc'][:,JOINT_ID]);
        dq = np.squeeze(data['dq'][:,JOINT_ID]);
        tau = np.squeeze(data['tau'][:,JOINT_ID]);
        ctrl = np.squeeze(data['ctrl'][:,JOINT_ID]);
        current = np.squeeze(data['current'][:,JOINT_ID]);

except IOError:
    print "Impossible to read data file %s" % (data_folder+DATA_FILE_NAME);
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");
    
#''' Load sensors data from file '''
try:
    sensor_data = np.load(data_folder+'data.npz');
    acc = sensor_data['acc'];
    gyro = sensor_data['gyro'];
    forceLA = sensor_data['forceLA'];
    forceRA = sensor_data['forceRA'];
    forceLL = sensor_data['forceLL'];
    forceRL = sensor_data['forceRL'];
except IOError:
    print "Impossible to read data file %f" % (data_folder+'data.npz');
    sys.exit("Run script compress_identification_data.py to generate data file from tracer log files.");
    
if(len(data['enc'].shape)==1):
    enc = np.squeeze(data['enc']);
    dq = np.squeeze(data['dq']);
    ddq = np.squeeze(data['ddq']);
    tau = np.squeeze(data['tau']);
    ctrl = np.squeeze(data['ctrl']);

else:
    enc = np.squeeze(data['enc'][:,JOINT_ID]);
    dq = np.squeeze(data['dq'][:,JOINT_ID]);
    ddq = np.squeeze(data['ddq'][:,JOINT_ID]);
    tau = np.squeeze(data['tau'][:,JOINT_ID]);
    ctrl = np.squeeze(data['ctrl'][:,JOINT_ID]);
    
maskSaturation=np.logical_and( (current>-9.5) , (current<9.5) )
maskLowVel=np.logical_and( (dq>-0.001) , (dq<0.001) )
maskPosVel=(dq> 0.001)
maskNegVel=(dq<-0.001)

#~ embed()

enc     = enc    [maskSaturation];
dq      = dq     [maskSaturation];
ddq     = ddq    [maskSaturation];
tau     = tau    [maskSaturation];
ctrl    = ctrl   [maskSaturation];
current = current[maskSaturation];

maskLowVel = maskLowVel[maskSaturation]
maskPosVel = maskPosVel[maskSaturation]
maskNegVel = maskNegVel[maskSaturation]

#Ktau,Tau0 Identification
if(FOLDER_ID==1):
    #~ fs=1000
    #~ f, Cxy = signal.coherence(tau[maskPosVel], current[maskPosVel], fs, nperseg=2048)
    #~ plt.semilogy(f, Cxy)
    #~ plt.xlabel('frequency [Hz]')
    #~ plt.ylabel('Coherence')
    #~ plt.show()

    #~ fs=1000
    #~ f, Cxy = signal.coherence(tau, current, fs, nperseg=1024)
    #~ plt.semilogy(f, Cxy)
    #~ plt.xlabel('frequency [Hz]')
    #~ plt.ylabel('Coherence')
    #~ plt.show()
    #~ embed()

    m = len(dq);
    # remove high velocity
    maskConstAng = (abs (dq)<ZERO_VELOCITY_THRESHOLD)
    # erode to get only steady phases where velocity is constant 
    maskConstAng=ndimage.morphology.binary_erosion(maskConstAng,None,100)
    #~ plt.figure()
    #~ plt.plot(ddq);
    maskConstPosAng=np.logical_and( maskConstAng ,maskPosVel )
    maskConstNegAng=np.logical_and( maskConstAng ,maskNegVel ) 
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(enc); plt.ylabel('q')
        q_const=enc.copy()
        q_const[np.logical_not(maskConstAng)]=np.nan
        plt.plot(q_const); plt.ylabel('q_const')


    #~ Tau = Kt * I + tau0
    Q=np.vstack([np.ones(len(current[maskConstPosAng])),current[maskConstPosAng]])
    coef = solveLeastSquare(Q.T,tau[maskConstPosAng])
    tau0_pos=coef[0,0]
    Kt_pos=coef[1,0]

    #~ Tau = Kt * I + tau0
    Q=np.vstack([np.ones(len(current[maskConstNegAng])),current[maskConstNegAng]])
    coef = solveLeastSquare(Q.T,tau[maskConstNegAng])
    tau0_neg=coef[0,0]
    Kt_neg=coef[1,0]

    Kt=(Kt_neg+Kt_pos) / 2.0
    print 'k_tau+[%d] = %f \t k_tau-[%d] = %f \t k_tau[%d] = %f' % (JOINT_ID,Kt_pos,JOINT_ID,Kt_neg, JOINT_ID,Kt);
    print 'tau0+[%d] = %f \t tau0-[%d] = %f' % (JOINT_ID,tau0_pos,JOINT_ID,tau0_neg);

    m = len(dq);
    #~ plt.plot(tau); plt.ylabel('Tau')              ;plt.figure()
    #~ plt.plot(current); plt.ylabel('Current')    ;plt.figure()
    #~ plt.plot(dq); plt.ylabel('dq')              ;plt.figure()
    #~ plt.plot(tau,current,'.');  plt.xlabel('Tau'); plt.ylabel('Current');
    plt.figure()
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.xlabel('Tau'); plt.ylabel('Current');
    
    plt.plot([Kt_pos*min(tau[maskConstPosAng])+tau0_pos, Kt_pos*max(tau[maskConstPosAng])+tau0_pos],[min(tau[maskConstPosAng]),max(tau[maskConstPosAng])],'g-',lw=2)
    plt.plot([Kt_neg*min(tau[maskConstNegAng])+tau0_neg, Kt_neg*max(tau[maskConstNegAng])+tau0_neg],[min(tau[maskConstNegAng]),max(tau[maskConstNegAng])],'g-',lw=2)
    
    #~ plt.plot(tau[maskLowVel],current[maskLowVel],'.',lw=3,markersize=1,c='0.5');  
    #~ plt.plot(tau[maskPosVel],current[maskPosVel],'rx',lw=3,markersize=1); 
    #~ plt.plot(tau[maskNegVel],current[maskNegVel],'bx',lw=3,markersize=1); 
    plt.plot(tau[maskLowVel],current[maskLowVel],'.',lw=3,markersize=1,c='0.5');  
    plt.plot(tau[maskConstPosAng],current[maskConstPosAng],'rx',lw=3,markersize=1); 
    plt.plot(tau[maskConstNegAng],current[maskConstNegAng],'bx',lw=3,markersize=1); 

    plt.plot([tau0_pos,tau0_neg],[0,0],'g-',lw=6,markersize=1); 
    
    plt.plot(tau[maskLowVel],current[maskLowVel],'.',lw=3,markersize=1,c='0.5');  
    
    #~ plt.figure()
    #~ plt.plot(tau[maskLowVel],ctrl[maskLowVel],'.');  plt.xlabel('Tau'); plt.ylabel('Current_ctrl');
    #~ plt.plot(tau[maskPosVel],ctrl[maskPosVel],'.');  plt.xlabel('Tau'); plt.ylabel('Current_ctrl');
    #~ plt.plot(tau[maskNegVel],ctrl[maskNegVel],'.');  plt.xlabel('Tau'); plt.ylabel('Current_ctrl');

    #~ plt.figure()
    #~ current=signal.medfilt(current, 201)
    #~ tau=signal.medfilt(tau, 201)
    #~ 
    #~ plt.plot(tau[maskLowVel],current[maskLowVel],'.');  plt.xlabel('Tau'); plt.ylabel('Current');
    #~ plt.plot(tau[maskPosVel],current[maskPosVel],'.');  plt.xlabel('Tau'); plt.ylabel('Current');
    #~ plt.plot(tau[maskNegVel],current[maskNegVel],'.');  plt.xlabel('Tau'); plt.ylabel('Current');
    plt.show()


#Kd Identification 
if(FOLDER_ID==2):
    #Filter current
    win = signal.hann(100)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    current = filtered_current


    m = len(dq);
    # remove high acceleration
    maskConstVel = (abs (ddq)<ZERO_ACCELERATION_THRESHOLD)
    # erode to get only steady phases where velocity is constant 
    maskConstVel=ndimage.morphology.binary_erosion(maskConstVel,None,100)
    #~ plt.figure()
    #~ plt.plot(ddq);
    maskConstPosVel=np.logical_and( maskConstVel ,maskPosVel )
    maskConstNegVel=np.logical_and( maskConstVel ,maskNegVel ) 
    
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(dq); plt.ylabel('dq')
        dq_const=dq.copy()
        dq_const[np.logical_not(maskConstVel)]=np.nan
        plt.plot(dq_const); plt.ylabel('dq_const')
    
    
    #~ i = Kv * dq + i_f
    Q=np.vstack([np.ones(len(dq[maskConstPosVel])),dq[maskConstPosVel]])
    coef = solveLeastSquare(Q.T,current[maskConstPosVel])
    current_mod_ConstPosVel=Q.T*coef
    i_f_pos=coef[0,0]
    Kv_pos=coef[1,0]

    #~ i = Kv * dq - i_f
    Q=np.vstack([np.ones(len(dq[maskConstNegVel])),dq[maskConstNegVel]])
    coef = solveLeastSquare(Q.T,current[maskConstNegVel])
    current_mod_ConstNegVel=Q.T*coef
    i_f_neg=coef[0,0]
    Kv_neg=coef[1,0]
    

    print 'k_v+[%d] = %f \t k_v-[%d] = %f' % (JOINT_ID ,Kv_pos,JOINT_ID, Kv_neg);
    print 'i_f+[%d] = %f \t i_f-[%d] = %f' % (JOINT_ID,i_f_pos,JOINT_ID,i_f_neg);



    
    plt.figure()
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.xlabel('Current'); plt.ylabel('dq');
    plt.plot(current,dq,'rx',lw=3,markersize=1,c='0.5');
    plt.plot([Kv_pos*max(dq)+i_f_pos, i_f_pos],[max(dq),0.0],'g-')
    plt.plot([Kv_neg*min(dq)+i_f_neg, i_f_neg],[min(dq),0.0],'g-')
    
    plt.plot(current[maskConstPosVel],dq[maskConstPosVel],'bx' ,lw=3,markersize=1); 
    plt.plot(current_mod_ConstPosVel ,dq[maskConstPosVel],'b--',lw=4,markersize=1);
    plt.plot(current[maskConstNegVel],dq[maskConstNegVel],'rx' ,lw=3,markersize=1); 
    plt.plot(current_mod_ConstNegVel ,dq[maskConstNegVel],'r--',lw=4,markersize=1); 

    #check 
    #k_tau+[16] = 1.403259 	 k_tau-[16] = 1.422349 	 k_tau[16] = 1.412804
    #tau0+[16] = -4.869321 	 tau0-[16] = 0.664924
    tau0= (4.869321 + 0.664924 )/2.0
    Kt=1.412804
    #k_tau+[16] = 1.342582 	 k_tau-[16] = 1.350970 	 k_tau[16] = 1.346776
    #tau0+[16] = -2.007070 	 tau0-[16] = 1.520840
    tau0= (2.007070 + 1.520840 )/2.0
    Kt=1.346776
    plt.plot([tau0 / Kt ,- tau0 / Kt], [0.0,0.0], 'ro')
    plt.plot([-2.007070 / Kt ,1.520840 / Kt], [0.0,0.0], 'go')
    
    print 'error betwin friction estimation on low vel and constant vel is %f %%' % (100 * ( tau0/Kt - (i_f_pos-i_f_neg)/2 )/(tau0/Kt) )
   
    plt.plot(current+tau/Kt,dq,'kx' ,lw=10,markersize=1); 
    
    #~ plt.figure()
    #~ plt.plot(ctrl[maskConstPosVel],dq[maskConstPosVel],'bx' ,lw=3,markersize=1); 
    #~ plt.plot(ctrl[maskConstNegVel],dq[maskConstNegVel],'rx' ,lw=3,markersize=1); 
    plt.show()

#J Identification
if(FOLDER_ID==3):
    COMPENSATE_TORQUE = True
    Kt=1.346776
    Kv_pos = 0.937689
    Kv_neg = 0.514448
    i_f_pos = 1.249641
    i_f_neg = -1.265535 
    JoverKt_pos = 0.204553 
    JoverKt_neg = 0.183804 

    #Filter current
    win = signal.hann(100)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    current = filtered_current

    m = len(dq);
    dt=0.001
    #~ # remove high jerk
    dddq = np.gradient(ddq,1)/dt
    maskConstAcc = (abs (dddq)<ZERO_JERK_THRESHOLD)
    #~ # erode to get only steady phases where acceleration is constant 
    maskConstAcc=ndimage.morphology.binary_erosion(maskConstAcc,None,100)
    plt.figure()

    maskConstPosAcc=np.logical_and( maskConstAcc ,maskPosVel )
    maskConstNegAcc=np.logical_and( maskConstAcc ,maskNegVel )
    
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(ddq); plt.ylabel('ddq')
        ddq_const=ddq.copy()
        ddq_const[np.logical_not(maskConstAcc)]=np.nan
        plt.plot(ddq_const); plt.ylabel('ddq_const')
        plt.show()


    #~ i(t) - Kd/Kt v(t) = J/Kt * ddq + b
    Q=np.vstack([np.ones(len(ddq[maskConstPosAcc])),ddq[maskConstPosAcc]])
    if COMPENSATE_TORQUE == True :
        coef = solveLeastSquare(Q.T,current[maskConstPosAcc]-(Kv_pos)*dq[maskConstPosAcc]-tau[maskConstPosAcc]/Kt)
    else :
        coef = solveLeastSquare(Q.T,current[maskConstPosAcc]-(Kv_pos)*dq[maskConstPosAcc])
    b_pos=coef[0,0]
    JoverKt_pos=coef[1,0]

    #~ i(t) - Kd/Kt v(t)  = J/Kt * ddq + b
    Q=np.vstack([np.ones(len(ddq[maskConstNegAcc])),ddq[maskConstNegAcc]])
    if COMPENSATE_TORQUE == True :
        coef = solveLeastSquare(Q.T,current[maskConstNegAcc]-(Kv_neg)*dq[maskConstNegAcc]-tau[maskConstNegAcc]/Kt) 
    else :
        coef = solveLeastSquare(Q.T,current[maskConstNegAcc]-(Kv_neg)*dq[maskConstNegAcc])
        
    b_neg=coef[0,0]
    JoverKt_neg=coef[1,0]
    
    print ' J/Kt+[%d] = %f ' % (JOINT_ID ,JoverKt_pos);
    print ' J/Kt-[%d] = %f ' % (JOINT_ID, JoverKt_neg);

    
    #~ plt.figure()
    #~ plt.plot(dq); plt.ylabel('dq')
    #~ dq_const=dq.copy()
    #~ dq_const[np.logical_not(maskConstVel)]=np.nan
    #~ plt.plot(dq_const); plt.ylabel('dq_const')
    
    plt.figure()
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    if COMPENSATE_TORQUE == True :
        plt.title(r'$i(t)-\frac{K_d}{K_t}\omega(t) = \frac{J}{K_t}a(t)+\frac{\tau(t)}{K_t}+b$')
        plt.xlabel(r'$ i(t)-\frac{Kd}{Kt}\omega(t)-\frac{\tau(t)}{K_t} $')
    else :
        plt.title(r'$i(t)-\frac{K_d}{K_t}\omega(t) = \frac{J}{K_t}a(t)+b$')
        plt.xlabel(r'$ i(t)-\frac{Kd}{Kt}\omega(t) $');
    plt.ylabel(r'$ acc(t)$');
    #~ plt.plot(current,dq,'rx',lw=3,markersize=1,c='0.5');
    id1=542
    id2=5245
    plt.plot([JoverKt_pos*max(ddq)+b_pos, 
              JoverKt_pos*min(ddq)+b_pos],
                         [max(ddq),
                          min(ddq)],'g-')

    plt.plot([JoverKt_neg*max(ddq)+b_neg, 
              JoverKt_neg*min(ddq)+b_neg],
                         [max(ddq),
                          min(ddq)],'g-')
    plt.plot(current[maskPosVel]-Kv_pos*dq[maskPosVel],ddq[maskPosVel],'rx',lw=3,markersize=1,c='0.5');
    plt.plot(current[maskNegVel]-Kv_neg*dq[maskNegVel],ddq[maskNegVel],'rx',lw=3,markersize=1,c='0.5');
    plt.plot(current[maskConstPosAcc]-(Kv_pos)*dq[maskConstPosAcc],ddq[maskConstPosAcc],'rx' ,lw=3,markersize=1); 
    plt.plot(current[maskConstNegAcc]-(Kv_neg)*dq[maskConstNegAcc],ddq[maskConstNegAcc],'bx' ,lw=3,markersize=1); 


    #with torque compensation:
    plt.plot(current[maskConstPosAcc]-Kv_pos*dq[maskConstPosAcc]-tau[maskConstPosAcc]/Kt,ddq[maskConstPosAcc],'rx',lw=1,markersize=1);
    plt.plot(current[maskConstNegAcc]-Kv_neg*dq[maskConstNegAcc]-tau[maskConstNegAcc]/Kt,ddq[maskConstNegAcc],'bx',lw=1,markersize=1);


    plt.figure()
    plt.plot(tau)
    plt.plot(dq)
    
    plt.show()

#model vs measurement
if(FOLDER_ID>0):
    tau_motor=np.zeros(len(tau))
    i_motor=np.zeros(len(current))
    
    for idx in range(len(tau)):
        tau_motor[idx]=motor.get_tau    (dq[idx],ddq[idx],current[idx])
        i_motor[idx]  =motor.get_current(dq[idx],ddq[idx],tau[idx])
        
    #~ plt.figure()
    #~ plt.plot(tau)
    #~ plt.plot(tau_motor)
    #~ plt.legend(['Estimated torque with dynamic model','Estimated torque with motor model'])
    
    plt.figure()
    plt.plot(current)
    plt.plot(i_motor)
    plt.legend(['measured current','Estimated current with model'])
    
    
    plt.show()
