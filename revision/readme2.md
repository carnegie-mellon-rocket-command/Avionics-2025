# setup guide (for arduino IDE)



# libraries
servo 1.1.2
SD 1.3
Adafruit bmp3xx 2.1.6
Adafruit LSM6DS
BasicLinearAlgebra 3.7
Kalman 1.1.0

Adafruit busio 1.17.0 ??


# launch settings
define SIMULATE false

# test settings
define SIMULATE true



setup
1. test SD card connection
2. initalize IMU (LSM6DSOX) and altimeter (bmp3xx) sensors
3. test all sensors
4. test ATS
5. initalize Kalman matrixes

loop
1. get altitude
2. update altimeter is miswritten?

get measurement
- refresh sensors and update globals with latest measurements
returns a string of latest measurements
1. update altimeter
2. get altitude and acceleration data
3. filter raw data and store it in globals

filter data (hot mess)


changes
1. got rid of bmp.performreading before atsTEST
2. ensures loop runs at consistent rate per iterations (run_timer())


personal notes

bmp.perform_reading -- update temperature and pressure of  (public attributes of bmp obj)
1. do this before using other bmp functions or variables

getEvent(&accel, &gyro, &temp);  
1. we use 3 sensor_event_t structs for clarity and ease of pointers even though we won't see all attributes
   and we can technically accomplish this with 1 sensor_event_t

To ask

we can't tell difference between `ledlogging` and `ledflying`
do we want to normalize acceleration or not? `readIMU`