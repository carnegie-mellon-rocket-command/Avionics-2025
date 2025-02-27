// ***************** LIBRARIES *****************

#include <Servo.h>
#include <SD.h>

// sensor libraries
#include <Adafruit_BMP3XX.h> //version 1.1.2
#include <Adafruit_LSM6DSOX.h>

#include <BasicLinearAlgebra.h> //version 3.7
#include <Kalman.h>

// ***************** Meta  *****************
//⚠⚠⚠ VERY IMPORTANT ⚠⚠⚠
// true sets subscale altitude target, false sets fullscale altitude target
#define SUBSCALE false

// ⚠⚠⚠ IMPORTANT⚠⚠⚠ 
//true will NOT actually gather data, only simulate it for testing purposes  
// false will gather data, FOR LAUNCH
#define SIMULATE false

// Simulation mode libraries
#if SIMULATE
    #include <Dictionary.h>
    Dictionary *simulatedSensorValues = new Dictionary();
#endif

// ⚠⚠⚠ Do not create variables with same name as linalg library ⚠⚠⚠ 
using namespace BLA;

// ***************** UNITS (in IPS) *****************
#define SEALEVELPRESSURE_HPA 1013.25f
#define METERS_TO_FEET 3.28084f
#define ATMOSPHERE_FLUID_DENSITY 0.076474f // lbs/ft^3
#define g 32.174f // ft/s^2
// ***************** Constants *****************
#define ROCKET_DRAG_COEFFICIENT 0.46f   // Average value from OpenRocket
#define ROCKET_CROSS_SECTIONAL_AREA 0.08814130888f // The surface area (ft^2) of the rocket facing upwards
#if SUBSCALE
    #define ROCKET_MASS 13.35f // lbs in dry mass (with engine housing but NOT propellant, assuming no ballast)
#else
    #define ROCKET_MASS 17.8125f // lbs in dry mass (with engine housing but NOT propellant, assuming no ballast)
#endif
#define MAX_FLAP_SURFACE_AREA 0.02930555555f
// #define ROCKET_MASS 19.5625f // lbs in dry mass (with engine housing but NOT propellant)
#define ATS_MAX_SURFACE_AREA 0.02930555555 + ROCKET_CROSS_SECTIONAL_AREA // The maximum surface area (ft^2) of the rocket with flaps extended, including rocket's area
#define TARGET_ACCELERATION 43.42135f
// Kalman filter parameters
#define NumStates 3
#define NumObservations 2
#define AltimeterNoise 1.0  // TODO: change
#define IMUNoise 1.0   // TODO: change
// Model covariance (TODO: change)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8
// ***************** GLOBALS *****************
#define SKIP_ATS false    // whether the rocket is NOT running ATS, so don't try to mount servos, etc.

// FLIGHT PARAMETERS
// Whether to print debugging messages to the serial monitor (even if SIMULATE is off)
const bool DEBUG = true;

// How frequently data should be collected (in milliseconds)
const int loop_target = 30;

// Target altitude in feet
#if SUBSCALE
    const float alt_target = 2000.0f; // ft
#else
    const float alt_target = 4500.0f; // ft
#endif

// Acceleration threshold for launch detection (ft/s^2)
const float accel_threshold = 2*g;
// Velocity threshold for landing detection (ft/s)
const float velocity_threshold = 0.1f;

// ***************** PIN DEFINITIONS *****************
const int ats_pin = 6;
const int LED_pin = LED_BUILTIN;
#if SUBSCALE
    const int altimeter_chip_select = 1;     // BMP
#else
    const int altimeter_chip_select = 10;     // BMP
#endif

// ATS SERVO PARAMETERS
Servo ATS_servo;
float ats_position = 0.0f;
const int ats_min = 0;
const int ats_max = 180;

// SD CARD PARAMETERS
#if SUBSCALE
  const int chip_select = 0;
#else
  const int chip_select = BUILTIN_SDCARD;
#endif
bool sd_active = false;
#if SUBSCALE
    String file_name = "subscl_2.txt"; // ⚠⚠⚠ FILE NAME MUST BE 8 CHARACTERS OR LESS OR ARDUINO CANNOT WRITE IT (WHY?!?!) ⚠⚠⚠
#else
    String file_name = "fullsc_1.txt";
#endif

// SENSOR OBJECTS

Adafruit_BMP3XX bmp;     // Altimeter
Adafruit_LSM6DSOX sox;    // IMU
sensors_event_t accel, gyro, temp;


// MEASUREMENT CONSTANTS AND VARIABLES

// Keeps track of data until it is written to the SD card
String buffer;

// Number of measurements to take before writing to SD card
const int buffer_size = 50;

// Keeps track of time to make sure we are taking measurements at a consistent rate
unsigned long start_time, curr_time, timer, loop_time_delta, prev_loop_time = 0;

// Filtered measurements shall be kept as global variables; raw data will be kept local to save memory
float altitude_filtered, velocity_filtered, acceleration_filtered;
// Internal stuff for the Kalman Filter
float altitude_filtered_previous, acceleration_filtered_previous = 0.0;
float gain_altitude, gain_acceleration, cov_altitude_current, cov_acceleration_current, cov_altitude_previous, cov_acceleration_previous = 0.0;
float variance_altitude, variance_acceleration = 0.1;     // Might want to change these based on experiments or by calculating in flight
// We don't necessarily need this variable at this point, but it will be used when more advanced filtering techniques are implemented
float previous_velocity_filtered = 0.0;

// Remembers if the rocket has launched and landed
bool launched, landed;
unsigned long launch_time;
unsigned long land_time = 0;

// Kalman filter stuff
BLA::Matrix<NumObservations> obs; // observation vector
KALMAN<NumStates,NumObservations> KalmanFilter; // Kalman filter
BLA::Matrix<NumStates> measurement_state;

