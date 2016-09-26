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
#from motor_model import Motor_model
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

#motor = Motor_model(tauf_pos, tauf_neg, Kt, Kv_pos, Kv_neg, J)
#~ motor.display()
# Test smooth tauf
#~ dqs=[]
#~ taufs=[]
#~ for dq in np.linspace(-1,1,1000):
    #~ dqs.append(dq)
    #~ taufs.append(motor.get_smooth_tauf(dq))
#~ plt.plot(dqs,taufs)
#~ plt.show()
Kt_p=np.array(30*(1.0,))
Kt_n=np.array(30*(1.0,))

Kf_p=np.array(30*(0.0,))
Kf_n=np.array(30*(0.0,))

Kv_p=np.array(30*(0.0,))
Kv_n=np.array(30*(0.0,))


# const pos
Kt_p[2] = 0.083160
Kt_n[2] = 0.078039
Kf_p[2] = 0.018288 #OSEF
Kf_n[2] = 1.018066 #OSEF

# const vel
Kv_p[2] = 0.327085
Kv_n[2] = 0.396856

Kf_p[2] = 0.132690
Kf_n[2] = 0.813251

''' Solve the least square problem:
    solve   y=ax+b in L2 norm
'''
def solve1stOrderLeastSquare(x,y):
    Q=np.vstack([np.ones(len(x)),x])
    coef = solveLeastSquare(Q.T,y)
    (a,b)=coef[1,0],coef[0,0]
    return (a,b);

''' Solve the least square problem:
    minimize   || A*x-b ||^2
'''
def solveLeastSquare(A, b):
    return np.linalg.pinv(A)*np.matrix(b).T;

DATA_SET  = 1;
FOLDER_ID = 1;
DATA_FILE_NAME = 'data.npz';

ZERO_VELOCITY_THRESHOLD     = 0.1
ZERO_ACCELERATION_THRESHOLD = 0.1
ZERO_JERK_THRESHOLD         = 3.0
SHOW_THRESHOLD_EFFECT = True

if(DATA_SET==1):
    if(FOLDER_ID==1):
        data_folder = '../../results/20160923_165916_Joint2_id_Kt/';
        JOINT_ID = 2;
    elif(FOLDER_ID==2):
        data_folder = '../../results/20160923_170659_Joint2_id_Kv/';
        JOINT_ID = 2;
    elif(FOLDER_ID==3):
        data_folder = '../../results/20160923_135235_Joint2_id_Ka/';
        JOINT_ID = 2;
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
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    current = filtered_current
    # Mask valid data***************************************************
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



    
    #~ y = a. x   +  b
    #~ i = Kt.tau + Kf
    #~ 
    # Identification ***************************************************
    y = current
    y_label = r'$i(t)$'
    x = tau
    x_label =r'$\tau(t)$'
    (a,b)=solve1stOrderLeastSquare(x[maskConstPosAng],y[maskConstPosAng])
    Ktp=a
    Kfp=b
    (a,b)=solve1stOrderLeastSquare(x[maskConstNegAng],y[maskConstNegAng])
    Ktn=a
    Kfn=-b
    
    
    # Plot *************************************************************
    plt.figure()    
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.plot(x     ,y     ,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAng],y[maskConstPosAng],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegAng],y[maskConstNegAng],'bx',lw=3,markersize=1); 
    #plot identified lin model
    plt.plot([min(x),max(x)],[Ktp*min(x)+Kfp ,Ktp*max(x)+Kfp],'g:',lw=3)
    plt.plot([min(x),max(x)],[Ktn*min(x)-Kfn ,Ktn*max(x)-Kfn],'g:',lw=3)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    
    
    print 'Kt_p[%d] = %f' % (JOINT_ID,Ktp);
    print 'Kt_n[%d] = %f' % (JOINT_ID,Ktn);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);
    plt.show()


#Kd Identification 
if(FOLDER_ID==2):
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    current = filtered_current

    # Mask valid data***************************************************
    m = len(dq);
    # remove high acceleration
    maskConstVel = np.logical_and( (abs (ddq)<ZERO_ACCELERATION_THRESHOLD) , (abs (dq)>ZERO_VELOCITY_THRESHOLD))
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


    
    #~ y        = a. x +  b
    #~ i-Kt.tau = Kv.dq + Kf
    #~ 
    # Identification ***************************************************
    y = current-Kt_p[JOINT_ID]*tau
    y[maskConstPosVel] = current[maskConstPosVel]-Kt_p[JOINT_ID]*tau[maskConstPosVel]
    y[maskConstNegVel] = current[maskConstNegVel]-Kt_n[JOINT_ID]*tau[maskConstNegVel]
    y_label = r'$i(t)-{K_t}{\tau(t)}$'
    x = dq
    x_label =r'$\dot{q}(t)$'
    (a,b)=solve1stOrderLeastSquare(x[maskConstPosVel],y[maskConstPosVel])
    Kvp=a
    Kfp=b
    (a,b)=solve1stOrderLeastSquare(x[maskConstNegVel],y[maskConstNegVel])
    Kvn=a
    Kfn=-b

    # Plot *************************************************************
    plt.figure()    
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.plot(x     ,y     ,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosVel],y[maskConstPosVel],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegVel],y[maskConstNegVel],'bx',lw=3,markersize=1); 
    #plot identified lin model
    plt.plot([0.0,max(dq)],[ Kfp,Kvp*max(dq)+Kfp],'g-')
    plt.plot([0.0,min(dq)],[-Kfn,Kvn*min(dq)-Kfn],'g-')
    plt.ylabel(y_label)
    plt.xlabel(x_label)

    print 'Kv_p[%d] = %f' % (JOINT_ID,Kvp);
    print 'Kv_n[%d] = %f' % (JOINT_ID,Kvn);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);
    plt.show()

#J Identification
if(FOLDER_ID==3):
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    current = filtered_current
    # Mask valid data***************************************************
    m = len(dq);
    dt=0.001
    #~ # remove high jerk
    dddq = np.gradient(ddq,1)/dt
    maskConstAcc = (abs (dddq)<ZERO_JERK_THRESHOLD)
    #~ # erode to get only steady phases where acceleration is constant 
    maskConstAcc=ndimage.morphology.binary_erosion(maskConstAcc,None,100)
    maskConstPosAcc=np.logical_and( maskConstAcc ,maskPosVel )
    maskConstNegAcc=np.logical_and( maskConstAcc ,maskNegVel )
    
    if SHOW_THRESHOLD_EFFECT :
        plt.figure()
        plt.plot(ddq); plt.ylabel('ddq')
        ddq_const=ddq.copy()
        ddq_const[np.logical_not(maskConstAcc)]=np.nan
        plt.plot(ddq_const); plt.ylabel('ddq_const')
        plt.show()

    #~ y              = a. x   +  b
    #~ i-Kt.tau-Kv.dq = Ka.ddq +  Kf
    #~ 
    # Identification ***************************************************
    y = current-Kt_p[JOINT_ID]*tau - Kv_p[JOINT_ID]*dq
    y[maskConstPosAcc] = current[maskConstPosAcc]-Kt_p[JOINT_ID]*tau[maskConstPosAcc] - Kv_p[JOINT_ID]*dq[maskConstPosAcc]
    y[maskConstNegAcc] = current[maskConstNegAcc]-Kt_p[JOINT_ID]*tau[maskConstNegAcc] - Kv_p[JOINT_ID]*dq[maskConstNegAcc]
    y_label = r'$i(t)-{K_t}{\tau(t)}-{K_v}{\dot{q}(t)}$'
    x = ddq
    x_label = r'$\ddot{q}(t)$'
    (a,b)=solve1stOrderLeastSquare(x[maskConstPosAcc],y[maskConstPosAcc])
    Kap=a
    Kfp=b
    (a,b)=solve1stOrderLeastSquare(x[maskConstNegAcc],y[maskConstNegAcc])
    Kan=a
    Kfn=-b
    
    # Plot *************************************************************
    plt.figure()    
    plt.axhline(0, color='black',lw=1)
    plt.axvline(0, color='black',lw=1)
    plt.plot(x     ,y     ,'.' ,lw=3,markersize=1,c='0.5');  
    plt.plot(x[maskConstPosAcc],y[maskConstPosAcc],'rx',lw=3,markersize=1); 
    plt.plot(x[maskConstNegAcc],y[maskConstNegAcc],'bx',lw=3,markersize=1); 
    #plot identified lin model
    plt.plot([min(x),max(x)],[Kap*min(x)+Kfp ,Kap*max(x)+Kfp],'g:',lw=3)
    plt.plot([min(x),max(x)],[Kan*min(x)-Kfn ,Kan*max(x)-Kfn],'g:',lw=3)
    plt.ylabel(y_label)
    plt.xlabel(x_label)

    print 'Ka_p[%d] = %f' % (JOINT_ID,Kap);
    print 'Ka_n[%d] = %f' % (JOINT_ID,Kan);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);
    plt.show()

    embed()
    
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
