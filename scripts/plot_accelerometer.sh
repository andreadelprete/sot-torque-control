BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='x imu acc','y imu acc','z imu acc' /accelerometer/data[0] /accelerometer/data[1] /accelerometer/data[2] &
