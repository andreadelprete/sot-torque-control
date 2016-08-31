BUFFER=30
REFRESH_RATE=10

rxplot --period=$BUFFER --buffer=$BUFFER --refresh_rate=$REFRESH_RATE --legend='x imu ang vel','y imu ang vel','z imu ang vel' /gyrometer/data[0] /gyrometer/data[1] /gyrometer/data[2] &
