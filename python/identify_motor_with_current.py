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
i(t) = Kt*tau(t) + Kv*dq(t) + Ka*ddq(t) + Kf*Sign(dq)
'''

#~ motor.display()
#~ # Test smooth tauf
#~ dqs=[]
#~ taufs=[]
#~ for dq in np.linspace(-1,1,1000):
    #~ dqs.append(dq)
    #~ taufs.append(motor.getCurrent(dq))
#~ plt.plot(dqs,taufs)
#~ plt.show()
jID = { "rhy" : 0,
        "rhr" : 1,
        "rhp" : 2,
        "rk"  : 3,
        "rap" : 4,
        "rar" : 5,
        "lhy" : 6,
        "lhr" : 7,
        "lhp" : 8,
        "lk"  : 9,
        "lap" : 10,
        "lar" : 11,
        "ty"  : 12,
        "tp"  : 13,
        "hy"  : 14,
        "hp"  : 15,
        "rsp" : 16,
        "rsr" : 17,
        "rsy" : 18,
        "re"  : 19,
        "rwy" : 20,
        "rwp" : 21,
        "rh"  : 22,
        "lsp" : 23,
        "lsr" : 24,
        "lsy" : 25,
        "le"  : 26,
        "lwy" : 27,
        "lwp" : 28,
        "lh"  : 29 }

Kt_p=np.array(30*(1.0,))
Kt_n=np.array(30*(1.0,))

Kf_p=np.array(30*(0.0,))
Kf_n=np.array(30*(0.0,))

Kv_p=np.array(30*(0.0,))
Kv_n=np.array(30*(0.0,))

Ka_p=np.array(30*(0.0,))
Ka_n=np.array(30*(0.0,))

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

# const acc
Ka_p[2] = 0.0
Ka_n[2] = 0.0

motor = Motor_model(Kt_p[2], Kt_n[2], 
                    Kf_p[2], Kf_n[2],
                    Kv_p[2], Kv_n[2],
                    Ka_p[2], Ka_n[2],0.1)


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


DATA_FILE_NAME = 'data.npz';

ZERO_VELOCITY_THRESHOLD     = 0.3
ZERO_ACCELERATION_THRESHOLD = 0.1
ZERO_JERK_THRESHOLD         = 3.0
SHOW_THRESHOLD_EFFECT = False

#~ JOINT_NAME = 'rhy'; 
#~ JOINT_NAME = 'rhr'; 
#~ JOINT_NAME = 'rhp'; 
#~ JOINT_NAME = 'rk'; 
#~ JOINT_NAME = 'rap'; 
#~ JOINT_NAME = 'rar'; 


JOINT_NAME = 'lhy'; # 6 ok
#~ JOINT_NAME = 'lhr'; # 7 NOK  
#~ JOINT_NAME = 'lhp'; # 8 ok
#~ JOINT_NAME = 'lk';  # 9  ok
#~ JOINT_NAME = 'lap'; # 10 ok
#~ JOINT_NAME = 'lar'; # 11 ok



#~ 
#~ IDENTIFICATION_MODE='static'
IDENTIFICATION_MODE='vel'
#~ IDENTIFICATION_MODE='acc'
#~ 


if(JOINT_NAME == 'rhy' ):
    data_folder_static = '../../results/20161114_135332_rhy_static/';
    data_folder_vel    = '../../results/20161114_143152_rhy_vel/';
    data_folder_acc    = '../../results/20161114_142351_rhy_acc/';
if(JOINT_NAME == 'rhr' ):
    data_folder_static = '../../results/20161114_144232_rhr_static/';
    data_folder_vel    = '../../results/20161114_150356_rhr_vel/';
    data_folder_acc    = '../../results/20161114_145456_rhr_acc/';
if(JOINT_NAME == 'rhp' ):
    data_folder_static = '../../results/20161114_150722_rhp_static/';
    data_folder_vel    = '../../results/20161114_151812_rhp_vel/';
    data_folder_acc    = '../../results/20161114_151259_rhp_acc/';
if(JOINT_NAME == 'rk' ):
    data_folder_static = '../../results/20161114_152140_rk_static/';
    data_folder_vel    = '../../results/20161114_153220_rk_vel/';
    data_folder_acc    = '../../results/20161114_152706_rk_acc/';
if(JOINT_NAME == 'rap' ):
    data_folder_static = '../../results/20161114_153739_rap_static/';
    data_folder_vel    = '../../results/20161114_154559_rap_vel/';
    data_folder_acc    = '../../results/20161114_154316_rap_acc/';
if(JOINT_NAME == 'rar' ):
    data_folder_static = '../../results/20161114_154945_rar_static/';
    data_folder_vel    = '../../results/20161114_160038_rar_vel/';
    data_folder_acc    = '../../results/20161114_155545_rar_acc/';





if(JOINT_NAME == 'lhy' ):
    data_folder_static = '../../results/20170113_144220_lhy_static/';
    data_folder_vel    = '../../results//';
    data_folder_acc    = '../../results/20170113_144710_lhy_const_acc/';
if(JOINT_NAME == 'lhr' ):
    data_folder_static = '../../results/20170113_145227_lhr_static/';
    data_folder_vel    = '../../results/20170113_150215_lhr_const_vel/';
    data_folder_acc    = '../../results/20170113_145826_lhr_const_acc/';
if(JOINT_NAME == 'lhp' ):
    data_folder_static = '../../results/20170113_150628_lhp_static/';
    data_folder_vel    = '../../results/20170113_151433_lhp_const_vel/';
    data_folder_acc    = '../../results/20170113_151103_lhp_const_acc/';
if(JOINT_NAME == 'lk' ):
    data_folder_static = '../../results/20170113_151748_lk_static/';
    data_folder_vel    = '../../results/20170113_152924_lk_const_vel/';
    data_folder_acc    = '../../results/20170113_152606_lk_const_acc/';
if(JOINT_NAME == 'lap' ):
    data_folder_static = '../../results/20170113_154007_lap_static/';
    data_folder_vel    = '../../results/20170113_154834_lap_const_vel/';
    data_folder_acc    = '../../results/20170113_154303_lap_const_acc/';
if(JOINT_NAME == 'lar' ):
    data_folder_static = '../../results/20170113_155150_lar_static/';
    data_folder_vel    = '../../results/20170113_160057_lar_const_vel/';
    data_folder_acc    = '../../results/20170113_155706_lar_const_acc/';


    
    
    
if (IDENTIFICATION_MODE=='static') : data_folder = data_folder_static
if (IDENTIFICATION_MODE=='vel')    : data_folder = data_folder_vel
if (IDENTIFICATION_MODE=='acc')    : data_folder = data_folder_acc



#~ 
#~ JOINT_NAME = 'rhr'; data_folder = '../../results/20161114_144232_rhr_static/'; IDENTIFICATION_MODE='static'
#~ JOINT_NAME = 'rhr'; data_folder = '../../results/20161114_145456_rhr_acc/';    IDENTIFICATION_MODE='acc'    
#~ JOINT_NAME = 'rhr'; data_folder = '../../results/20161114_150356_rhr_vel/';    IDENTIFICATION_MODE='vel'
#~ 
#~ JOINT_NAME = 'rhp'; data_folder = '../../results/20161114_150722_rhp_static/'; IDENTIFICATION_MODE='static'
#~ JOINT_NAME = 'rhp'; data_folder = '../../results/20161114_151259_rhp_acc/';    IDENTIFICATION_MODE='acc'    
#~ JOINT_NAME = 'rhp'; data_folder = '../../results/20161114_151812_rhp_vel/';    IDENTIFICATION_MODE='vel'
#~ 
#~ JOINT_NAME = 'rk' ; data_folder = '../../results/20161114_152140_rk_static/';  IDENTIFICATION_MODE='static'
#~ JOINT_NAME = 'rk' ; data_folder = '../../results/20161114_152706_rk_acc/';     IDENTIFICATION_MODE='acc'    
#~ JOINT_NAME = 'rk' ; data_folder = '../../results/20161114_153220_rk_vel/';     IDENTIFICATION_MODE='vel'
#~ 
#~ JOINT_NAME = 'rap'; data_folder = '../../results/20161114_153739_rap_static/'; IDENTIFICATION_MODE='static'
#~ JOINT_NAME = 'rap'; data_folder = '../../results/20161114_154316_rap_acc/';    IDENTIFICATION_MODE='acc'    
#~ JOINT_NAME = 'rap'; data_folder = '../../results/20161114_154559_rap_vel/';    IDENTIFICATION_MODE='vel'
#~ 
#~ 
#~ JOINT_NAME = 'rar'; data_folder = '../../results/20161114_154945_rar_static/'; IDENTIFICATION_MODE='static'
#~ JOINT_NAME = 'rar'; data_folder = '../../results/20161114_155545_rar_acc/';    IDENTIFICATION_MODE='acc'    
#~ JOINT_NAME = 'rar'; data_folder = '../../results/20161114_160038_rar_vel/';    IDENTIFICATION_MODE='vel'
#~ 
#~ JOINT_NAME = 'rhy'; data_folder = '../../results/20161114_135332_rhy_static/'; IDENTIFICATION_MODE='static'
#~ JOINT_NAME = 'rhy'; data_folder = '../../results/20161114_142351_rhy_acc/';    IDENTIFICATION_MODE='acc'    
#~ JOINT_NAME = 'rhy'; data_folder = '../../results/20161114_143152_rhy_vel/';    IDENTIFICATION_MODE='vel'


JOINT_ID = jID[JOINT_NAME]

'''
20161114_144232_rhr_static           
20161114_145456_rhr_acc          
20161114_150356_rhr_vel     
      
20161114_150722_rhp_static   
20161114_151259_rhp_acc
20161114_151812_rhp_vel     
     
20161114_152140_rk_static        
20161114_152706_rk_acc            
20161114_153220_rk_vel     
     
20161114_153739_rap_static           
20161114_154316_rap_acc          
20161114_154559_rap_vel    
  
20161114_154945_rar_static         
20161114_155545_rar_acc           
20161114_160038_rar_vel

20161114_135332_rhy_static 
20161114_142351_rhy_acc
20161114_143152_rhy_vel                
'''

   
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
maskPosVel=(dq> 0.001)
maskNegVel=(dq<-0.001)

#~ embed()

enc     = enc    [maskSaturation];
dq      = dq     [maskSaturation];
ddq     = ddq    [maskSaturation];
tau     = tau    [maskSaturation];
ctrl    = ctrl   [maskSaturation];
current = current[maskSaturation];

maskPosVel = maskPosVel[maskSaturation]
maskNegVel = maskNegVel[maskSaturation]

#Ktau,Tau0 Identification
if(IDENTIFICATION_MODE=='static'):
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same')/sum(win)
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
    plt.title('Static experiment - Joint '+JOINT_NAME)
    print 'Kt_p[%d] = %f' % (JOINT_ID,Ktp);
    print 'Kt_n[%d] = %f' % (JOINT_ID,Ktn);
    print 'Kf_p[%d] = %f' % (JOINT_ID,Kfp);
    print 'Kf_n[%d] = %f' % (JOINT_ID,Kfn);
    #save parameters for next identification level**********************
    np.savez(data_folder+'motor_param_'+JOINT_NAME+'.npz',Ktp=Ktp,Ktn=Ktn)
    plt.savefig(data_folder+"static_"+JOINT_NAME+".jpg")
    plt.show()

#Kd Identification 
if(IDENTIFICATION_MODE=='vel'):
    #load parameters from last identification level*********************
    try:
        data_motor_param = np.load(data_folder_static+'motor_param_'+JOINT_NAME+'.npz')
        Kt_p[JOINT_ID]=(data_motor_param['Ktp'].item())
        Kt_n[JOINT_ID]=(data_motor_param['Ktn'].item())
    except IOError:
        print "Impossible to read data file %s" % (data_folder_static+'motor_param_'+JOINT_NAME+'.npz');
        sys.exit("Run identification on static experiments.");
    
    #Filter current*****************************************************
    win = signal.hann(10)
    filtered_current = signal.convolve(current, win, mode='same') / sum(win)
    #~ plt.plot(current)
    #~ plt.plot(filtered_current)
    #~ current = filtered_current
    #~ plt.show()
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
    plt.savefig(data_folder+"vel_"+JOINT_NAME+".jpg")
    plt.show()

#J Identification
if(IDENTIFICATION_MODE=='acc'):
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

#model vs measurement
if(False):
    tau_motor=np.zeros(len(tau))
    i_motor=np.zeros(len(current))
    
    for idx in range(len(tau)):
        tau_motor[idx]=motor.getTorque    (current[idx], dq[idx], ddq[idx])
        i_motor[idx]  =motor.getCurrent   (tau[idx],     dq[idx], ddq[idx])
        
    plt.figure()
    plt.plot(tau)
    plt.plot(tau_motor)
    plt.legend(['Estimated torque with dynamic model','Estimated torque with motor model'])
    
    plt.figure()
    plt.plot(current)
    plt.plot(i_motor)
    plt.legend(['measured current','Estimated current with model'])
    
    
    plt.show()
