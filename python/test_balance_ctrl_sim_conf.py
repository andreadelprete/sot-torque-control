from math import sqrt
import numpy as np

''' *********************** USER-PARAMETERS *********************** '''

''' INITIAL STATE PARAMETERS '''
MAX_TEST_DURATION           = 3000;
dt                          = 1e-3;
model_path                  = ["/home/adelpret/devel/sot_hydro/install/share"];
urdfFileName                = model_path[0] + "/hrp2_14_description/urdf/hrp2_14_reduced.urdf";
freeFlyer                   = True;
q0_urdf = np.matrix([0.0, 0.0, 0.648702, 0.0, 0.0 , 0.0, 1.0,                             # Free flyer 0-6
                     0.0, 0.0, 0.0, 0.0,                                                  # CHEST HEAD 7-10
                     0.261799388,  0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # LARM       11-17
                     0.261799388, -0.174532925, 0.0, -0.523598776, 0.0, 0.0, 0.174532925, # RARM       18-24
                     0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # LLEG       25-30
                     0.0, 0.0, -0.453785606, 0.872664626, -0.41887902, 0.0,               # RLEG       31-36
                     ]).T;
q0_sot = (  # Free flyer
            0., 0., 0.643702, 0., 0. , 0.,
            # Legs
            0., 0., -0.453786, 0.872665, -0.418879, 0.,
            0., 0., -0.453786, 0.872665, -0.418879, 0.,
            # Chest and head
            0., 0., 0., 0.,
            # Arms
            0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,
            0.261799, 0.17453,  0., -0.523599, 0., 0., 0.1);
COM_DES = (0.01, 0.0, 0.81);
          
v0 = np.matrix(np.zeros(36)).T;
active_joints = (1, 1, 1, 1, 1, 1,          # lleg
                 1, 1, 1, 1, 1, 1,          # rleg
                 0, 0, 0, 0,                # chest-head
                 0, 0, 0, 0, 0, 0, 0,       # larm
                 0, 0, 0, 0, 0, 0, 0)       # rarm

GEAR_RATIOS = (384.0, 240.0, 180.0, 200.0, 180.0, 100.0,
               384.0, 240.0, 180.0, 200.0, 180.0, 100.0,
               207.69, 381.54, 100.0, 100.0,
               219.23, 231.25, 266.67, 250.0, 145.45, 350.0, 200.0,
               219.23, 231.25, 266.67, 250.0, 145.45, 350.0, 200.0);
ROTOR_INERTIAS = (1.01e-4, 6.96e-4, 1.34e-4, 1.34e-4, 6.96e-4, 6.96e-4,
                  1.01e-4, 6.96e-4, 1.34e-4, 1.34e-4, 6.96e-4, 6.96e-4,
                  6.96e-4, 6.96e-4, 1.10e-4, 1.10e-4,
                  6.96e-4, 6.60e-4, 1.00e-4, 6.60e-4, 1.10e-4, 1.00e-4, 1.00e-4, 
                  6.96e-4, 6.60e-4, 1.00e-4, 6.60e-4, 1.10e-4, 1.00e-4, 1.00e-4);

''' CONTROLLER CONFIGURATION '''
ENABLE_CAPTURE_POINT_LIMITS     = False;
ENABLE_TORQUE_LIMITS            = True;
ENABLE_FORCE_LIMITS             = True;
ENABLE_JOINT_LIMITS             = True;
IMPOSE_POSITION_BOUNDS          = True;
IMPOSE_VELOCITY_BOUNDS          = True;
IMPOSE_VIABILITY_BOUNDS         = True;
IMPOSE_ACCELERATION_BOUNDS      = True;
JOINT_POS_PREVIEW               = 1.5; # preview window to convert joint pos limits into joint acc limits
JOINT_VEL_PREVIEW               = 1;   # preview window to convert joint vel limits into joint acc limits
MAX_JOINT_ACC                   = 30.0;
MAX_MIN_JOINT_ACC               = 10.0;
USE_JOINT_VELOCITY_ESTIMATOR    = False;
ACCOUNT_FOR_ROTOR_INERTIAS      = True;

# CONTROLLER GAINS
kp_posture  = 10.0;   # proportional gain of postural task
kd_posture  = 2*sqrt(kp_posture);
kp_pos      = 100.0;   # proportional gain of position controller
kd_pos      = 2*sqrt(kp_pos);
kp_constr   = 100.0;   # constraint proportional feedback gain
kd_constr   = 2*sqrt(kp_constr);   # constraint derivative feedback gain
kp_com      = 10.0;
kd_com      = 2*sqrt(kp_com);
constraint_mask = np.array([True, True, True, True, True, True]).T;
ee_mask         = np.array([True, True, True, True, True, True]).T;

# CONTROLLER WEIGTHS
w_com               = 1.0;
w_posture           = 1e-2;  # weight of postural task
w_forces            = 1e-4;
w_base_orientation  = 0.0;
w_torques           = 0.0;

# QP SOLVER PARAMETERS
maxIter = 300;      # max number of iterations
maxTime = 0.8;      # max computation time for the solver in seconds
verb=0;             # verbosity level (0, 1, or 2)

# CONTACT PARAMETERS
RIGHT_FOOT_SIZES  = (0.130,  -0.100,  0.056,  -0.075); # pos x, neg x, pos y, neg y size 
LEFT_FOOT_SIZES = (0.130, -0.100,  0.075, -0.056); # pos x, neg x, pos y, neg y size 

RIGHT_FOOT_SIZES  = (0.130,  -0.100,  0.056,  -0.056); # pos x, neg x, pos y, neg y size 
RIGHT_FOOT_CONTACT_POINTS  = ((RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[0], RIGHT_FOOT_SIZES[1], RIGHT_FOOT_SIZES[1]),
                              (RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2], RIGHT_FOOT_SIZES[3], RIGHT_FOOT_SIZES[2]),
                              (-0.105, -0.105, -0.105, -0.105));    # contact points in local reference frame

LEFT_FOOT_CONTACT_POINTS  = np.matrix([[LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[3], -0.105],
                                     [LEFT_FOOT_SIZES[0], LEFT_FOOT_SIZES[2], -0.105],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[3], -0.105],
                                     [LEFT_FOOT_SIZES[1], LEFT_FOOT_SIZES[2], -0.105]]).T    # contact points in local reference frame
mu  = np.array([0.3, 0.1]);          # force and moment friction coefficient
fMin = 1e-3;					     # minimum normal force

''' SIMULATOR PARAMETERS '''
FORCE_TORQUE_LIMITS            = ENABLE_TORQUE_LIMITS;
FORCE_JOINT_LIMITS             = ENABLE_JOINT_LIMITS and IMPOSE_POSITION_BOUNDS;
USE_LCP_SOLVER                 = False

''' STOPPING CRITERIA THRESHOLDS '''
MAX_CONSTRAINT_ERROR        = 0.1;

''' INITIAL STATE PARAMETERS '''
INITIAL_CONFIG_ID                   = 0;
INITIAL_CONFIG_FILENAME             = '../../../data/hrp2_configs_coplanar';

''' VIEWER PARAMETERS '''
ENABLE_VIEWER               = True;
PLAY_MOTION_WHILE_COMPUTING = True;
PLAY_MOTION_AT_THE_END      = True;
DT_VIEWER                   = 10*dt;   # timestep used to display motion with viewer
SHOW_VIEWER_FLOOR           = True;

''' FIGURE PARAMETERS '''
SAVE_FIGURES     = False;
SHOW_FIGURES     = False;
SHOW_LEGENDS     = True;
LINE_ALPHA       = 0.7;
#BUTTON_PRESS_TIMEOUT        = 100.0;
