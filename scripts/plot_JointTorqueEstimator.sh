BUFFER=30
REFRESH_RATE=10
export ROS_MASTER_URI=http://hrp2014c:11311/
#rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='fx LLEG','fx RLEG','fy LLEG','fy RLEG','fz LLEG','fz RLEG' /estimator_contactWrenchLeftFoot/data[0],/estimator_contactWrenchRightFoot/data[0] /estimator_contactWrenchLeftFoot/data[1],/estimator_contactWrenchRightFoot/data[1] /estimator_contactWrenchLeftFoot/data[2],/estimator_contactWrenchRightFoot/data[2] &
rqt_plot /estimator_jointsTorques/data[2]  /estimator_jointsTorquesFromInertiaModel/data[2]  /estimator_jointsTorquesFromMotorModel/data[2] 
