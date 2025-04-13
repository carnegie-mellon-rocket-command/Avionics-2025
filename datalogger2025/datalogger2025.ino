/*
Carnegie Schmellon Rocketry Club: Precision INstrumented Experimental Aerial Propulsion Payload 
                                  for Low-altitude Exploration ("PINEAPPLE") data logger             
(see what I did there lol)

Made by the 2025 Avionics team :D
*/

// ***************** LIBRARIES *****************
#include <Servo.h>
#include <SD.h>
#include <Adafruit_BMP3XX.h> //version 1.1.2
#include <Adafruit_LSM6DSOX.h>
#include <BasicLinearAlgebra.h> //version 3.7
#include <Kalman.h>
#include <cassert>

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
    Dictionary *m_simSensorValues = new Dictionary();
#endif

// Assertions (for debugging)
#if ENABLE_ASSERTS
    #define requires(condition) assert(condition)
    #define ensures(condition) assert(condition)
#else
    #define requires(condition) ((void)0)
    #define ensures(condition) ((void)0)
#endif
// ⚠⚠⚠ Do not create variables with same name as linalg library ⚠⚠⚠ 
using namespace BLA;

// ***************** UNITS (in IPS) *****************
#define SEA_LEVEL_PRESSURE_HPA 1013.25f
#define METERS_TO_FEET 3.28084f
#define ATMOSPHERE_FLUID_DENSITY 0.076474f // lbs/ft^3
#define GRAVITY 32.174f // ft/s^2
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
#define ENABLE_ASSERTS true
// FLIGHT PARAMETERS
// Whether to print debugging messages to the serial monitor (even if SIMULATE is off)
const bool DEBUG = false;

// How frequently data should be collected (in milliseconds)
const int LOOP_TARGET_MS = 30;

// Target altitude in feet
#if SUBSCALE
    const float ALT_TARGET = 2000.0f; // ft
#else
    const float ALT_TARGET = 4500.0f; // ft above launch pad
#endif

// Acceleration threshold for launch detection (ft/s^2)
const float ACCEL_THRESHOLD = 3*GRAVITY;
// Velocity threshold for landing detection (ft/s)
const float VELOCITY_THRESHOLD = 0.1f;


// ***************** PIN DEFINITIONS *****************
const int ATS_PIN = 6;
const int LED_PIN = LED_BUILTIN;
#if SUBSCALE
    const int altimeter_chip_select = 1;     // BMP
#else
    const int altimeter_chip_select = 10;     // BMP
#endif

// ATS SERVO PARAMETERS
Servo m_atsServo;
float gAtsPosition = 0.0f;
const int ATS_MIN = 0;
const int ATS_MAX = 180;
const float ATS_IN = 0.0f;
const float ATS_OUT = 1.0f;
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
    String file_name = "PDF.txt";
#endif

// SENSOR OBJECTS

Adafruit_BMP3XX m_bmp;     // Altimeter
Adafruit_LSM6DSOX m_sox;    // IMU
sensors_event_t accel, gyro, temp;

// MEASUREMENT VARIABLES

// Keeps track of data until it is written to the SD card
String gBuffer;

// Number of measurements to take before writing to SD card
const int buffer_size = 50;

// Keeps track of time to make sure we are taking measurements at a consistent rate
unsigned long gStartTime, gCurrTime, gTimer, gTimeDelta, gPrevLoopTime = 0;

// Filtered measurements shall be kept as global variables; raw data will be kept local to save memory
float gAltFiltered, gVelocityFiltered, gAccelFiltered, gPredictedAltitude;
// Internal stuff for the Kalman Filter
float altitude_filtered_previous, acceleration_filtered_previous = 0.0;
float gain_altitude, gain_acceleration, cov_altitude_current, cov_acceleration_current, cov_altitude_previous, cov_acceleration_previous = 0.0;
float variance_altitude, variance_acceleration = 0.1;     // Might want to change these based on experiments or by calculating in flight
// We don't necessarily need this variable at this point, but it will be used when more advanced filtering techniques are implemented
float previous_velocity_filtered = 0.0;

// Remembers if the rocket has launched and landed
bool gLaunched, gLanded;
unsigned long gLaunchTime;
unsigned long land_time = 0;
float absolute_alt_target = ALT_TARGET;

// Kalman filter stuff
BLA::Matrix<NumObservations> obs; // observation vector
KALMAN<NumStates,NumObservations> KalmanFilter; // Kalman filter
BLA::Matrix<NumStates> measurement_state;

// Entry point to the program
/** @brief Initializes all devices, test devices and ATS */ 
void setup() {
    LEDSetup();
    // Setup serial terminal
    Serial.begin(115200);
    Serial.println("Initializing...");

    // Initalize Simulator
    // #if SIMULATE
    //     startSimulation();
    // #endif

    // Initialize SD card
    if (!InitializeSDCard()) {
        Serial.println("SD card setup failed. Aborting.");
        LEDError();
    }

    // Initialize sensors
    if (!SetupSensors()) {
        Serial.println("Sensor setup failed. Aborting.");
        LEDError();
    }
    
    // Test if all sensors on Altimeter works
    if (!TestSensors()){
        Serial.println("one or more altimeter sensors are not working");
        LEDError();
    }
    
    Serial.println("All sensors working!");
    TestATS();

    // Initalize time evolution matrix
    KalmanFilter.F = {1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};

    // measurement matrix (first row: altimeter, second row: accelerometer)
    KalmanFilter.H = {1.0, 0.0, 0.0,
                      0.0, 0.0, 1.0};
    // measurement covariance matrix
    KalmanFilter.R = {AltimeterNoise*AltimeterNoise,   0.0,
                                    0.0, IMUNoise*IMUNoise};
    // model covariance matrix
    KalmanFilter.Q = {m_p*m_p,     0.0,   0.0,
                        0.0,  m_s*m_s,     0.0,
                        0.0,     0.0, m_a*m_a};

    obs.Fill(0.0);
    measurement_state.Fill(0.0);

    Serial.println("Arduino is ready!");
    LEDSuccess();
    gStartTime = millis();
}

// Repeats indefinitely after setup() is finished
/** @brief collect/log data  and control ATS */
void loop() {
    gBuffer = "";
    for (int i = 0; i < buffer_size; i++) {
        RunTimer();  // ensures loop runs at a consistent rate
        // Get measurements from sensors and add to buffer
        gBuffer = gBuffer + GetMeasurements() + "\n";
        // ****(Pre-Flight)****
        // Detect launch based on acceleration threshold
        if (gAccelFiltered > ACCEL_THRESHOLD && !gLaunched) {
            // Write CSV header to the file
            WriteData("***************** START OF DATA ***************** TIME SINCE BOOT: " + String(millis()) + " ***************** TICK SPEED: " + String(LOOP_TARGET_MS) + "ms\n");
            WriteData("time, pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, gAltFiltered (ft), gVelocityFiltered (ft/s), gAccelFiltered (ft/s^2), temperature (from IMU, degrees C), gAtsPosition (servo degrees), gAltPredicted (ft)\n");

            if (DEBUG) {Serial.println("Rocket has launched!");}
            gLaunched = true;
            gLaunchTime = millis();
            
            // Bring the ATS back online
            attachATS();
            setATSPosition(ATS_IN);

            // Set status LED
            LEDLogging();
        }
        // ****(During Flight)****
        // If the rocket has launched, adjust the ATS as necessary, and detect whether the rocket has landed
        if (gLaunched) {
            LEDFlying();
            AdjustATS();
            if (DetectLanding()) {gLanded = true;}
        }
        else {
            // If we are still on the pad, measure the altitude of the launch pad
            absolute_alt_target = ALT_TARGET + gAltFiltered;
        }
    }
    if (gLaunched) {WriteData(gBuffer);}
    // ****(END OF FLIGHT)****
    if (gLanded) {
        // End the program
        detachATS();
        if (DEBUG) {Serial.println("Rocket has landed, ending program");}
        while (true);
    }
}

/** @brief ensures data collected at same rate
 * ensures each iteration in arduino main loop for loop runs at same rate */
void RunTimer() {
    long tempTime = millis() - gPrevLoopTime;
    // Serial.println(tempTime);
    if (tempTime < LOOP_TARGET_MS) {
        delayMicroseconds((LOOP_TARGET_MS - tempTime) * 1000);
    }
    else if (tempTime > LOOP_TARGET_MS + 1) {
        Serial.println("Board is unable to keep up with target loop time of " + String(LOOP_TARGET_MS) + " ms (execution took "+ String(tempTime) + " ms)");
    }
    gCurrTime = millis();
    gTimer = gCurrTime - gStartTime;
    gTimeDelta = gCurrTime - gPrevLoopTime;
    // Serial.println(loop_time);
    gPrevLoopTime = gCurrTime;
}

/** @brief Initialize the SD card  
* Right now, tries to connect 10 times before going into an error state; could change so it keeps trying indefinitely
*/
bool InitializeSDCard() {
    #if SIMULATE
        // If simulation mode is active, don't try to connect to any hardware
        Serial.println("(simulation) SD card initialized successfully!");
        return true;

        // Use this instead to simulate pain (pain is realistic)
        // return false;
    #endif

    if (DEBUG) {Serial.print("Initializing SD card...");}

    for (int i = 0; i < 10; i++) {
        if (SD.begin(chip_select)) {
            break;
        }
        if (DEBUG) {
            Serial.println("SD card initialization failed. Trying again...");
        }
        delay(1000);
        if (i == 9) {
            Serial.println("SD card initialization failed 10 times. Aborting.");
            return false;
        }
    }

    if (DEBUG) {Serial.println("SD card initialized successfully!");}

    sd_active = true;
    return true;
}

/** @brief Write data to SD card in "datalog.txt" file */
void WriteData(String text) {
    #if SIMULATE
        Serial.println("(simulation) Data to SD card: " + text);
        return;
    #endif

    if (sd_active) {
        File data_file = SD.open("subscl_1.txt", FILE_WRITE);
        if (data_file) {
            if (DEBUG) {Serial.println("Writing to SD card!");}
            data_file.print(text);
            data_file.close();
        } else {
            // Could leave this out so the program tries to keep logging data even if it fails once
            // sd_active = false;

            Serial.println("Error opening subscl_1.txt");
        }
    } else {
        // If the SD card has not connected successfully
        if (DEBUG) {Serial.println("SD logging failed. Continuing without logging.");}
    }
}

// ***************** Sensor Setup *****************
/**  @brief Initialize all sensors */
bool SetupSensors() {
  #if SIMULATE
    Serial.println("(simulation) Sensors connected successfully!");
    return true;
  #endif

  if (setupLSM6DSOX() && setupBMP3XX()) {
    if (DEBUG) {Serial.println("Sensors initialized successfully!");}
    m_bmp.performReading();
    Serial.println("Setup reading fine");
    return true;
  }
  return false;
}

/**  @brief Setup Altimeter */
bool setupBMP3XX() {
  if (!m_bmp.begin_SPI(altimeter_chip_select)) {
    Serial.println("Unable to connect to altimeter");
    return false;
  }
  m_bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  m_bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  m_bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  m_bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("Altimeter good");
  m_bmp.performReading();
  Serial.print(m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA));
  return true;
}

/**  @brief Setup IMU */
bool setupLSM6DSOX() {
  if (!m_sox.begin_I2C()) {
    Serial.println("Unable to connect to IMU");
    return false;
  }
  return true;
}

/**  @brief Test Altimeter and IMU */
bool TestSensors() {
    #if SIMULATE
        return true;
    #endif
    if (!m_bmp.performReading()){
        Serial.println("one or more altimeter sensors are not working");
        return false;
    }
    if (!m_sox.getEvent(&accel, &gyro, &temp)){
        Serial.println("one or more IMU sensors are not working");
        return false;
    }
    return true;
}

// ***************** Collecting/Logging Data *****************
/**  @brief  Read altitude from altimeter
* IMPORTANT: update altimeter with `performreading()` beforehand
* @returns raw altitude (meters)
*/
float ReadAltimeter() {
    float altimeter_data = m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA) * METERS_TO_FEET;
    if (DEBUG) {Serial.println("Altimeter: " + String(altimeter_data));}
    return altimeter_data;
}

/** @brief Read acceleration from IMU 
 * IMPORTANT: update IMU with `getevent()` first for current reading
 * @returns raw acceleration (ft/s^2)
*/
float ReadIMU() {
    float imu_data = pow(pow(accel.acceleration.x,2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2), 0.5);
    // Convert to feet
    imu_data = imu_data * METERS_TO_FEET;
    if (DEBUG) {Serial.println("IMU: " + String(imu_data));}
    return imu_data;
}

/** @brief Read temperature 
 * reads from IMU
 * IMPORTANT: update IMU with `getevent()` beforehand
 * @returns temperature (degrees C)
 */
float ReadThermometer() {
    float thermometer_data = temp.temperature;
    if (DEBUG) {Serial.println("Thermometer: " + String(thermometer_data));}
    return thermometer_data;
}

/** @brief refresh sensors and update globals with latest measurements
 * output string can be viewed when DEBUG is true
 * @returns a CSV string with format:
 * - time, pressure (hPa), altitude_raw (m), acceleration_raw_x (m/s^2), 
 * - acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, 
 * - gAltFiltered, gVelocityFiltered, gAccelFiltered, 
 * - temperature (from IMU, degrees C), gAtsPosition (degrees)
 */
String GetMeasurements() {
    #if SIMULATE
        // If simulation mode is ON, get simulated data from serial bus instead
        return getSimulatedMeasurements();
    #endif
    //update sensors
    m_bmp.performReading(); //update altimeter
    m_sox.getEvent(&accel, &gyro, &temp); //update imu
    
    //pass raw data to be filtered
    FilterData(ReadAltimeter(), ReadIMU()); 

    // print measurements
    String movementData = String(m_bmp.pressure/100.0) + "," + 
                           String(m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA)) + "," + 
                           String(accel.acceleration.x*METERS_TO_FEET) + "," + 
                           String(accel.acceleration.y*METERS_TO_FEET) + "," + 
                           String(accel.acceleration.z*METERS_TO_FEET) + "," + 
                           String(gyro.gyro.x) + "," + 
                           String(gyro.gyro.y) + "," + 
                           String(gyro.gyro.z) + "," + 
                           String(gAltFiltered) + "," + 
                           String(gVelocityFiltered) + "," + 
                           String(gAccelFiltered);
    String timeData = String(millis() - gStartTime);
    String sensorData = String(ReadThermometer());
    // if (DEBUG) {Serial.println(timeData + "," + movementData + "," + sensorData + "," + String(gAtsPosition));}
    return timeData + "," + movementData + "," + sensorData + "," + String(gAtsPosition) + "," + String(gPredictedAltitude);
}

/** @brief Filter raw data and updates globals
 * still need to implement */ 
void FilterData(float alt, float acc) {
    float DT = ((float)gTimeDelta)/1000;

    KalmanFilter.F = {1.0,  DT,  DT*DT/2,
		 0.0, 1.0,       DT,
         0.0, 0.0,      1.0};

    measurement_state(0) = (double)alt;
    measurement_state(1) = (double)0.0;
    measurement_state(2) = (double)acc;

    obs = KalmanFilter.H * measurement_state;
    KalmanFilter.update(obs);
    // BLA::Matrix<NumObservations, 3> current_obs = {alt, 0.0, 0.0,
    //                                             0.0, 0.0, acc};
    // KalmanFilter.update(current_obs);
    gAltFiltered = KalmanFilter.x(0);
    gVelocityFiltered = KalmanFilter.x(1);
    gAccelFiltered = KalmanFilter.x(2);
    // Serial << alt << "," << acc <<"," << gAltFiltered << "," << gVelocityFiltered << "," << gAccelFiltered << "\n";
    Serial << "Current accel: " << gAccelFiltered << "\n";
    // Serial.println("Filtered Altitude: " + String(gAltFiltered) + " Filtered Velocity: " + String(gVelocityFiltered) + " Filtered Acceleration: " + String(gAccelFiltered));
}

/** @brief Detect if rocket has landed 
 * if the rocket has been reasonably still for 5 seconds, it is considered landed
*/
bool DetectLanding() {
    if (gLaunchTime != 0 && millis() - gLaunchTime > 60000) { return true; } 
    return false;
}

/** @brief runs pid
 * @arg error: acceleration error
 * @arg Kp: proportional gain
 * @arg Kd: derivative gain
 * - positive error means rocket is going too slow
 * - negative error means rocket is going too fast
 * - Kp: higher = stronger immediate response
 * - Kd: adjusts response based on how frequently error changes
 * @return the absolute position to adjust ATS to in degrees
 */
float PIDFactor(int error, float Kp, float Kd)
{
  static int old_error = 0;

  float proportional = error * Kp;

  float derivative = (error - old_error) * Kd;
  old_error = error;

  return proportional + derivative; 
}

// ***************** ATS METHODS *****************

/** @brief turn on ATS servo */
void attachATS() {
    requires(!m_atsServo.attached());
    // #if SIMULATE
    //     return;
    // #endif
    #if SKIP_ATS
        return;
    #endif
    m_atsServo.attach(ATS_PIN);
}
/** @brief turn off ATS servo
 * (for saving power) */
void detachATS() {
    requires(m_atsServo.attached());
    // #if SIMULATE
    //     return;
    // #endif
    #if SKIP_ATS
        return;
    #endif
    m_atsServo.detach();
}

/** @brief set ATS position  
 * @arg percent_rot: float between 0 (fully in) and 1 (fully out)
 * only call this function when servo position changes
 */
void setATSPosition(float percent_rot) {
    // #if SIMULATE
    //     return;
    // #endif
    #if SKIP_ATS
        return;
    #endif
    float pos = (ATS_MAX - ATS_MIN) * (1.0f - percent_rot) + ATS_MIN;
    m_atsServo.write(int(pos));
    // if (DEBUG) {Serial.println("ATS position set to " + String(pos));}
    Serial.println("ATS position set to " + String(percent_rot)); // for debugging
}


/** @brief adjust ATS with PID
 * Adjust the ATS based on the current altitude and desired apogee (stored in absolute_alt_target; the ALT_TARGET represents altitude above the launch level, while absolute_alt_target is the altitude above sea level)
 */
void AdjustATS() {
    float targetAcceleration = abs(pow(gVelocityFiltered,2)/(2*(absolute_alt_target-gAltFiltered)));
     // Retract ATS fully after 18 seconds
    if (millis() - gLaunchTime > 18000) {
        setATSPosition(ATS_IN);
        delay(10);
        return;
    }
    
    // Fully deploy ATS if reached Altitude target
    if (gAltFiltered >= absolute_alt_target) {
        gAtsPosition = ATS_OUT;
    }
    else {
        // Calculate desired surface-area to reach target altitude
        float target_area = (pow(gVelocityFiltered, 2)/(absolute_alt_target - gAltFiltered) - 2*GRAVITY)*ROCKET_MASS/(gVelocityFiltered*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT);
        Serial.println("Old ATS pos: " + String(target_area/ATS_MAX_SURFACE_AREA));
        // Calculate error in acceleration
        float error = gAccelFiltered - targetAcceleration; // positive if drag + gravity >= target, means if <, we are going TOO FAST and need to slow down
        // calculate adjustment
        float adjustment;
        if (error > 0){ //too slow
            adjustment = 0;
        } 
        else { //too fast
            adjustment = PIDFactor(abs(error), 0.03,0); // should normalize to 0 to 1
        }
        gAtsPosition = adjustment;
        gPredictedAltitude = (0.5*ROCKET_MASS*pow(gVelocityFiltered, 2))/(ROCKET_MASS*GRAVITY + 0.5*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT*pow(gVelocityFiltered,2)*ATS_MAX_SURFACE_AREA*adjustment);
        //debug
        Serial.println("the predicted altitude: "+ String(gPredictedAltitude));
    }

    // ATS window
    if (millis() - gLaunchTime > 4500) {
        if (millis() - gLaunchTime < 6000) {
          // Adjust ATS based on position
          setATSPosition(ATS_IN);
          delay(500);
          setATSPosition(ATS_OUT);  // Initial position
          delay(500);
          setATSPosition(ATS_IN);  // Reset position
          delay(500);
          Serial.println("Cycling ATS position");
        } else {
        // Adjust ATS /based on position
          setATSPosition(gAtsPosition);
          Serial.println("ATS position: " + String(gAtsPosition));
        }
    }
}

/** @brief Test the ATS: fully extend and retract it, then detach the servo */
void TestATS() {
    if (DEBUG) { Serial.println("Testing ATS..."); }
    attachATS();
    setATSPosition(ATS_IN);
    delay(2000);
    setATSPosition(ATS_OUT);  // Initial position
    delay(2000);
    setATSPosition(ATS_IN);  // Reset position
    delay(2000);
    detachATS();
    Serial.println("ATS Test Sucessful...");
}

// ***************** STATUS LED Methods *****************
void LEDLogging() {
    digitalWrite(LED_PIN, HIGH);
}

void LEDSuccess() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_PIN, LOW);
        delay(250);
        digitalWrite(LED_PIN, HIGH);
        delay(250);
    }
}

void LEDError() {
    while (true) {
        digitalWrite(LED_PIN, HIGH);
        delay(1000);
        digitalWrite(LED_PIN, LOW);
        delay(1000);
    }
}

void LEDFlying() {
    digitalWrite(LED_PIN, HIGH);
}

void LEDSetup(){
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}

// If in simulation mode, update simulated sensor values from serial bus
#if SIMULATE

// // Begin the simulation by waiting for the host program to start and give confirmation over serial
// void startSimulation() {
//     m_simSensorValues->insert("altitude_raw", "0.0");
//     m_simSensorValues->insert("acceleration_raw", "0.0");
//     m_simSensorValues->insert("temperature", "0.0");

//     // Await connection from host computer
//     while (!Serial.readString()) {
//         delay(10);
//     }

//     Serial.println("(simulation) Arduino is ready!");
// }

// // Reads the serial buffer for the most recent data sent by the hosts, and updates the m_simSensorValues hashmap accordingly
// void collectSimulatedData() {
//     String serialBuffer = Serial.readString();
//     String currentKey, currentValue = "";
//     for (int i = 0; i < serialBuffer.length(); i++) {
//         if (serialBuffer[i] == ':') {
//             currentKey = currentValue;
//             currentValue = "";
//         } else if (serialBuffer[i] == ',') {
//             m_simSensorValues->insert(currentKey, currentValue);
//             Serial.println("Received "+currentKey+", "+currentValue);
//             currentKey = "";
//             currentValue = "";
//         } else {
//             currentValue += serialBuffer[i];
//         }
//     }
// }

// float dataAsFloat(String measurement_name) {
//     return m_simSensorValues->search(measurement_name).toFloat();
// }

int i=0;
int j=0;
float altitude_raw[] = {283.66, 283.63, 283.66, 283.67, 283.65, 283.71, 283.78, 283.87, 283.89, 283.91, 283.95, 283.97, 284.0, 284.01, 284.05, 284.03, 284.02, 283.98, 284.01, 284.08, 284.12, 284.13, 284.15, 284.14, 284.07, 284.11, 284.11, 284.12, 284.17, 284.26, 284.18, 284.18, 284.12, 284.04, 283.93, 283.87, 283.85, 283.83, 283.82, 283.8, 283.78, 283.76, 283.77, 283.77, 283.75, 283.69, 284.01, 284.15, 284.3, 284.49, 285.43, 286.0, 286.56, 287.36, 288.28, 289.11, 290.04, 290.88, 291.91, 292.93, 294.23, 295.86, 297.5, 299.01, 300.51, 301.94, 303.37, 304.8, 306.35, 307.75, 308.91, 310.06, 311.49, 313.13, 314.9, 316.9, 319.07, 321.2, 323.29, 325.52, 327.88, 330.58, 333.51, 336.4, 339.17, 342.26, 345.34, 348.63, 352.09, 355.87, 360.23, 365.08, 371.01, 378.06, 385.56, 392.73, 402.06, 409.99, 417.49, 424.62, 437.04, 444.22, 451.67, 458.79, 465.72, 472.07, 478.33, 484.3, 489.94, 495.8, 501.54, 507.06, 512.55, 517.93, 523.16, 528.13, 533.67, 537.15, 540.54, 546.09, 552.02, 557.81, 563.92, 569.99, 575.81, 581.37, 586.68, 590.72, 594.15, 598.37, 603.38, 609.23, 615.21, 621.05, 626.77, 632.28, 637.67, 642.9, 648.06, 653.02, 657.78, 662.53, 667.14, 671.8, 676.65, 681.5, 686.45, 691.51, 696.63, 701.49, 711.41, 716.2, 720.85, 725.28, 729.58, 733.82, 738.08, 742.32, 746.62, 750.84, 755.16, 759.6, 763.85, 768.28, 772.65, 777.05, 781.44, 785.82, 790.19, 794.08, 798.33, 802.84, 807.38, 811.96, 816.53, 820.98, 825.5, 829.87, 834.01, 838.08, 842.22, 846.39, 850.75, 855.11, 859.6, 863.93, 868.1, 872.36, 876.62, 880.64, 884.79, 888.96, 893.18, 897.25, 901.25, 905.37, 909.34, 912.77, 914.88, 914.29, 908.42, 905.6, 904.04, 903.35, 903.24, 903.35, 903.83, 904.85, 906.19, 907.71, 909.81, 912.35, 915.02, 917.8, 920.94, 924.26, 928.59, 933.95, 938.42, 942.6, 946.35, 950.12, 953.9, 958.22, 962.61, 967.33, 972.07, 976.75, 981.51, 986.03, 989.89, 993.56, 997.21, 1000.41, 1003.63, 1007.0, 1010.86, 1014.92, 1019.27, 1023.6, 1027.68, 1031.5, 1034.75, 1038.25, 1042.02, 1045.99, 1050.1, 1054.11, 1058.01, 1062.12, 1070.42, 1074.22, 1077.81, 1081.28, 1084.5, 1087.93, 1091.16, 1094.36, 1097.66, 1100.82, 1103.79, 1107.1, 1110.52, 1113.93, 1117.43, 1120.75, 1124.09, 1127.3, 1130.45, 1133.54, 1136.59, 1139.6, 1142.85, 1145.54, 1148.81, 1152.04, 1155.32, 1158.4, 1161.63, 1164.76, 1167.76, 1170.98, 1173.97, 1177.13, 1180.16, 1183.16, 1186.02, 1188.87, 1191.83, 1194.8, 1197.79, 1200.72, 1203.56, 1206.44, 1209.37, 1212.31, 1215.07, 1217.97, 1220.46, 1223.29, 1229.79, 1233.29, 1236.66, 1239.27, 1242.02, 1244.43, 1247.1, 1249.74, 1252.18, 1254.79, 1257.27, 1259.79, 1262.32, 1264.66, 1266.92, 1269.16, 1271.94, 1274.69, 1277.06, 1279.57, 1281.95, 1284.31, 1286.81, 1289.3, 1291.38, 1293.71, 1296.06, 1298.66, 1301.2, 1303.86, 1306.55, 1309.0, 1311.38, 1313.89, 1316.27, 1318.7, 1321.09, 1323.37, 1325.72, 1327.89, 1329.91, 1332.11, 1334.49, 1336.74, 1338.89, 1341.28, 1343.68, 1346.0, 1348.08, 1350.27, 1354.27, 1356.23, 1358.25, 1360.29, 1362.14, 1364.04, 1366.14, 1368.12, 1370.02, 1371.93, 1373.78, 1375.7, 1377.66, 1379.61, 1381.51, 1383.4, 1385.2, 1387.17, 1389.11, 1391.15, 1392.95, 1394.92, 1396.76, 1398.29, 1400.08, 1401.87, 1403.83, 1405.51, 1407.17, 1409.16, 1410.97, 1412.8, 1414.68, 1416.5, 1418.28, 1419.95, 1421.65, 1423.3, 1424.92, 1426.64, 1428.33, 1430.02, 1431.68, 1433.29, 1434.77, 1436.44, 1437.94, 1439.49, 1441.01, 1442.67, 1446.21, 1448.09, 1449.63, 1451.2, 1452.68, 1454.16, 1455.64, 1456.95, 1458.3, 1459.59, 1460.88, 1462.19, 1463.51, 1464.82, 1466.15, 1467.58, 1468.79, 1470.04, 1471.19, 1472.48, 1473.76, 1474.95, 1476.28, 1477.51, 1478.79, 1480.16, 1481.38, 1482.5, 1483.76, 1484.97, 1486.18, 1487.24, 1488.3, 1489.51, 1490.67, 1491.82, 1492.96, 1494.01, 1494.96, 1496.13, 1497.28, 1498.29, 1499.37, 1500.43, 1501.41, 1502.28, 1503.38, 1504.33, 1505.26, 1506.25, 1508.28, 1509.3, 1510.03, 1510.92, 1511.73, 1512.46, 1513.25, 1514.0, 1514.89, 1515.76, 1516.57, 1517.39, 1518.16, 1519.05, 1519.71, 1520.42, 1521.13, 1521.81, 1522.55, 1523.37, 1524.15, 1524.89, 1525.58, 1526.25, 1527.04, 1527.77, 1528.49, 1529.09, 1529.7, 1530.32, 1530.84, 1531.5, 1532.05, 1532.59, 1533.15, 1533.68, 1534.25, 1534.93, 1535.53, 1536.12, 1536.74, 1537.24, 1537.83, 1538.33, 1538.87, 1539.42, 1539.86, 1540.32, 1540.74, 1541.25, 1542.26, 1542.8, 1543.35, 1543.73, 1544.26, 1544.62, 1544.87, 1545.21, 1545.5, 1545.85, 1546.09, 1546.38, 1546.69, 1547.04, 1547.33, 1547.6, 1547.85, 1548.06, 1548.24, 1548.43, 1548.67, 1548.97, 1549.12, 1549.41, 1549.52, 1549.75, 1549.92, 1550.02, 1550.17, 1550.28, 1550.43, 1550.59, 1550.75, 1550.9, 1550.99, 1551.07, 1551.06, 1551.15, 1551.16, 1551.24, 1551.29, 1551.39, 1551.44, 1457.24, 1473.05, 1495.28, 1512.92, 1527.37, 1538.05, 1547.79, 1559.75, 1562.22, 1563.17, 1565.31, 1568.4, 1569.79, 1572.32, 1573.45, 1573.92, 1574.24, 1574.4, 1575.25, 1575.87, 1576.7, 1578.2, 1578.31, 1575.27, 1574.6, 1573.22, 1572.01, 1570.78, 1569.98, 1569.29, 1568.96, 1569.64, 1569.83, 1569.71, 1569.19, 1571.29, 1573.93, 1574.49, 1574.59, 1574.82, 1574.95, 1575.63, 1576.01, 1576.57, 1577.34, 1577.61, 1577.59, 1577.78, 1577.7, 1577.62, 1578.0, 1577.8, 1577.13, 1576.11, 1575.09, 1573.85, 1572.32, 1566.89, 1563.73, 1561.27, 1558.96, 1557.08, 1555.96, 1555.02, 1554.07, 1552.97, 1551.9, 1550.72, 1549.38, 1548.23, 1547.24, 1546.22, 1545.71, 1545.32, 1545.26, 1545.75, 1546.74, 1547.37, 1545.86, 1544.72, 1543.01, 1541.31, 1540.22, 1539.53, 1538.97, 1538.44, 1538.63, 1539.9, 1542.06, 1545.13, 1546.35, 1544.79, 1544.23, 1546.04, 1546.63, 1547.56, 1547.92, 1547.4, 1545.46, 1542.4, 1538.4, 1534.9, 1533.23, 1532.69, 1532.4, 1531.46, 1530.34, 1530.75, 1530.74, 1530.25, 1530.13, 1529.73, 1528.75, 1528.27, 1527.54, 1526.52, 1526.4, 1525.97, 1524.94, 1523.44, 1522.19, 1521.19, 1520.42, 1520.01, 1519.64, 1519.43, 1519.19, 1518.5, 1517.61, 1516.33, 1514.6, 1512.85, 1511.39, 1510.37, 1509.8, 1510.34, 1511.43, 1512.2, 1512.21, 1511.76, 1510.93, 1510.38, 1510.84, 1510.37, 1509.81, 1509.38, 1508.72, 1508.65, 1508.52, 1508.22, 1507.8, 1506.97, 1505.12, 1503.12, 1501.49, 1501.84, 1503.48, 1506.33, 1505.98, 1505.55, 1504.61, 1503.41, 1502.75, 1503.25, 1503.53, 1503.11, 1501.13, 1499.01, 1496.69, 1495.6, 1495.9, 1496.15, 1496.3, 1496.48, 1497.68, 1499.5, 1499.37, 1498.3, 1497.08, 1495.49, 1494.13, 1493.78, 1493.01, 1491.98, 1490.32, 1487.76, 1485.26, 1484.17, 1483.93, 1484.11, 1484.55, 1484.7, 1485.09, 1484.94, 1483.58, 1481.25, 1479.41, 1478.33, 1477.44, 1476.15, 1474.43, 1472.58, 1471.08, 1471.77, 1472.53, 1472.86, 1471.86, 1468.25, 1466.23, 1464.34, 1463.13, 1462.52, 1461.92, 1461.45, 1460.81, 1460.11, 1459.43, 1458.62, 1458.52, 1458.51, 1458.55, 1458.57, 1458.63, 1458.86, 1459.42, 1459.82, 1459.78, 1459.72, 1459.32, 1458.3, 1457.43, 1456.49, 1455.45, 1455.28, 1455.06, 1454.42, 1453.57, 1453.02, 1452.61, 1452.15, 1451.18, 1450.1, 1449.15, 1448.19, 1447.68, 1447.02, 1446.51, 1445.84, 1445.08, 1443.58, 1441.31, 1438.43, 1435.68, 1433.36, 1432.2, 1431.44, 1432.2, 1434.13, 1433.41, 1432.66, 1431.53, 1430.77, 1429.89, 1428.93, 1427.1, 1425.61, 1424.53, 1424.48, 1426.69, 1424.72, 1422.0, 1416.75, 1412.21, 1409.09, 1406.79, 1405.53, 1405.84, 1406.43, 1407.49, 1407.65, 1407.17, 1406.86, 1406.22, 1405.4, 1404.03, 1403.24, 1402.88, 1402.05, 1400.91, 1399.51, 1398.26, 1397.46, 1396.62, 1396.0, 1394.6, 1393.71, 1393.08, 1393.08, 1393.65, 1393.91, 1393.74, 1392.83, 1391.99, 1390.34, 1388.3, 1386.89, 1385.5, 1383.83, 1381.92, 1379.73, 1377.67, 1376.04, 1374.42, 1373.09, 1371.83, 1370.94, 1369.86, 1369.01, 1368.2, 1367.64, 1367.14, 1367.2, 1367.57, 1369.41, 1371.67, 1372.78, 1372.67, 1372.19, 1371.24, 1370.25, 1369.02, 1366.82, 1364.59, 1362.1, 1360.2, 1358.89, 1357.55, 1356.66, 1356.2, 1355.98, 1356.18, 1356.39, 1356.37, 1356.22, 1356.13, 1356.04, 1355.51, 1354.45, 1353.08, 1351.52, 1350.22, 1349.05, 1347.9, 1347.18, 1347.8, 1348.1, 1347.81, 1346.34, 1344.62, 1342.53, 1340.5, 1338.49, 1336.93, 1335.89, 1335.33, 1335.11, 1334.7, 1334.04, 1332.97, 1331.63, 1330.31, 1328.86, 1328.01, 1327.72, 1328.22, 1328.72, 1329.43, 1329.77, 1330.04, 1330.0, 1329.77, 1329.42, 1329.23, 1329.16, 1329.17, 1328.97, 1328.64, 1328.09, 1327.61, 1326.98, 1326.52, 1325.86, 1325.4, 1325.02, 1323.63, 1322.56, 1321.67, 1320.5, 1319.5, 1318.58, 1317.34, 1316.7, 1316.82, 1317.42, 1319.12, 1320.52, 1320.83, 1316.67, 1314.44, 1312.78, 1310.69, 1308.0, 1305.7, 1303.8, 1302.14, 1300.42, 1299.43, 1298.26, 1297.55, 1297.21, 1297.15, 1297.6, 1298.83, 1298.79, 1298.59, 1297.87, 1296.79, 1295.83, 1295.82, 1295.87, 1296.4, 1295.67, 1295.22, 1294.82, 1294.02, 1292.96, 1291.13, 1289.06, 1286.7, 1284.24, 1281.49, 1279.13, 1277.51, 1275.76, 1274.42, 1273.65, 1273.19, 1273.05, 1272.82, 1272.33, 1272.89, 1272.81, 1272.28, 1272.37, 1273.02, 1273.74, 1274.58, 1276.0, 1275.72, 1275.19, 1274.9, 1274.27, 1273.95, 1273.78, 1273.1, 1271.76, 1270.25, 1267.39, 1263.97, 1261.79, 1259.93, 1258.23, 1256.4, 1254.7, 1253.19, 1251.74, 1250.51, 1250.06, 1249.64, 1249.59, 1249.99, 1249.11, 1247.98, 1247.09, 1246.35, 1245.39, 1244.89, 1244.76, 1245.13, 1245.95, 1245.5, 1244.23, 1242.19, 1239.54, 1237.79, 1236.85, 1236.88, 1237.04, 1237.41, 1236.6, 1234.71, 1233.76, 1233.73, 1234.44, 1235.34, 1235.42, 1235.82, 1233.57, 1231.71, 1229.66, 1227.52, 1225.71, 1223.98, 1222.4, 1220.66, 1219.24, 1218.24, 1217.9, 1217.46, 1217.0, 1217.38, 1218.26, 1218.69, 1218.77, 1219.03, 1218.97, 1218.35, 1218.26, 1218.1, 1217.65, 1217.37, 1217.03, 1215.91, 1214.89, 1213.91, 1213.41, 1213.03, 1212.77, 1212.39, 1212.49, 1211.96, 1211.04, 1210.21, 1209.62, 1208.75, 1207.92, 1206.99, 1206.6, 1205.51, 1204.38, 1203.58, 1202.6, 1201.88, 1201.07, 1200.95, 1199.47, 1198.34, 1194.6, 1193.56, 1192.29, 1191.12, 1190.39, 1189.53, 1188.56, 1187.38, 1186.3, 1185.57, 1184.46, 1182.73, 1180.15, 1176.72, 1174.14, 1172.75, 1172.49, 1173.61, 1176.08, 1177.17, 1176.14, 1175.17, 1172.58, 1168.03, 1164.03, 1161.33, 1159.3, 1157.65, 1156.99, 1156.99, 1156.59, 1155.65, 1154.55, 1153.78, 1153.3, 1153.43, 1154.83, 1156.95, 1156.66, 1155.54, 1153.46, 1151.58, 1151.3, 1153.06, 1154.04, 1155.83, 1155.71, 1153.68, 1151.34, 1149.82, 1145.62, 1143.12, 1141.53, 1140.53, 1139.81, 1139.06, 1138.39, 1137.08, 1135.12, 1133.03, 1131.06, 1129.75, 1129.05, 1127.96, 1126.35, 1124.82, 1123.71, 1123.14, 1123.31, 1123.65, 1124.84, 1126.54, 1127.18, 1127.78, 1127.98, 1127.76, 1126.53, 1124.64, 1122.33, 1120.48, 1119.25, 1118.3, 1117.44, 1116.68, 1116.35, 1116.01, 1115.57, 1115.2, 1114.58, 1113.78, 1112.77, 1112.53, 1112.8, 1112.46, 1111.74, 1111.04, 1110.89, 1110.46, 1110.01, 1109.56, 1108.42, 1107.59, 1106.56, 1105.12, 1103.71, 1102.22, 1100.71, 1099.72, 1099.53, 1101.18, 1100.78, 1098.65, 1095.5, 1091.56, 1087.92, 1085.1, 1083.15, 1082.01, 1082.04, 1083.0, 1085.65, 1086.94, 1088.61, 1091.23, 1091.4, 1089.19, 1086.24, 1084.84, 1084.22, 1083.07, 1082.18, 1081.2, 1080.18, 1079.36, 1078.21, 1076.21, 1073.96, 1071.99, 1070.05, 1068.53, 1067.49, 1067.05, 1066.64, 1066.56, 1067.2, 1067.92, 1070.08, 1071.11, 1071.7, 1071.76, 1071.12, 1069.52, 1067.49, 1064.98, 1062.46, 1060.78, 1059.51, 1058.23, 1057.02, 1056.84, 1056.99, 1056.67, 1055.49, 1052.52, 1049.52, 1046.94, 1045.35, 1044.59, 1043.39, 1042.36, 1042.22, 1043.59, 1046.34, 1048.0, 1048.84, 1048.22, 1046.45, 1043.91, 1041.65, 1039.5, 1036.83, 1034.25, 1031.98, 1030.81, 1030.49, 1030.35, 1030.46, 1030.55, 1029.85, 1028.19, 1027.26, 1027.83, 1028.31, 1029.11, 1031.42, 1031.64, 1030.86, 1029.72, 1028.8, 1027.81, 1026.06, 1024.59, 1022.16, 1020.11, 1018.05, 1016.52, 1015.72, 1016.63, 1016.75, 1015.65, 1013.98, 1012.9, 1012.11, 1012.39, 1013.1, 1012.74, 1010.11, 1007.23, 1006.21, 1006.71, 1007.33, 1008.84, 1008.65, 1006.25, 1004.73, 1003.45, 1001.65, 998.77, 995.93, 995.35, 995.34, 995.02, 995.08, 994.25, 992.71, 992.23, 992.95, 992.48, 990.92, 988.68, 986.49, 984.95, 985.45, 986.72, 987.55, 988.19, 988.49, 987.85, 986.54, 985.2, 982.05, 980.12, 978.54, 977.43, 976.73, 976.14, 976.16, 976.09, 975.83, 975.15, 974.7, 974.42, 973.84, 972.89, 971.96, 971.0, 970.22, 969.86, 969.79, 969.56, 968.57, 966.68, 963.85, 961.32, 959.37, 958.22, 958.86, 960.17, 960.22, 959.98, 959.71, 958.96, 957.98, 954.57, 953.31, 952.91, 952.97, 952.98, 952.56, 952.45, 952.18, 951.63, 950.77, 948.76, 945.92, 943.36, 942.68, 942.23, 941.15, 939.05, 936.59, 937.28, 938.4, 937.55, 935.62, 934.24, 933.09, 931.9, 928.94, 925.94, 923.4, 921.7, 919.82, 918.44, 917.73, 917.54, 918.1, 919.37, 920.58, 922.18, 922.03, 921.71, 920.67, 918.91, 916.64, 914.97, 914.03, 913.84, 913.88, 914.12, 914.6, 914.61, 914.55, 914.2, 913.95, 913.44, 913.13, 912.82, 912.18, 911.54, 910.93, 910.38, 909.42, 908.58, 907.9, 907.28, 907.12, 907.19, 905.75, 902.0, 897.08, 894.84, 892.12, 889.53, 887.76, 886.85, 886.95, 887.5, 888.33, 888.22, 886.96, 885.73, 884.33, 881.7, 878.99, 876.85, 874.73, 873.16, 872.82, 873.22, 873.81, 875.23, 876.16, 875.43, 875.15, 874.93, 874.89, 874.66, 874.43, 874.4, 874.28, 874.37, 873.96, 873.23, 874.04, 874.41, 874.72, 874.39, 873.62, 873.2, 872.59, 872.17, 871.32, 870.46, 869.85, 869.05, 868.29, 867.25, 866.99, 867.19, 865.13, 863.13, 861.3, 859.8, 858.44, 856.97, 855.78, 854.95, 853.44, 851.7, 849.54, 847.01, 844.74, 842.63, 840.94, 839.57, 838.94, 839.11, 839.7, 839.67, 839.52, 839.21, 838.98, 839.35, 839.1, 838.93, 838.57, 835.49, 832.77, 830.83, 828.79, 827.08, 826.22, 826.11, 826.34, 827.45, 828.58, 828.77, 828.89, 828.45, 827.17, 826.34, 825.29, 824.61, 823.66, 822.6, 821.36, 820.72, 819.59, 818.28, 815.45, 814.93, 814.78, 814.28, 814.52, 815.65, 815.34, 813.82, 812.14, 810.06, 807.72, 805.54, 803.02, 801.33, 801.01, 801.08, 801.32, 801.51, 801.37, 801.36, 801.06, 800.69, 800.44, 800.0, 798.68, 797.08, 795.91, 795.1, 794.96, 794.64, 793.41, 792.62, 791.34, 788.8, 787.11, 786.78, 788.3, 790.82, 791.21, 790.36, 791.47, 792.03, 792.25, 791.21, 788.75, 784.85, 782.57, 782.3, 783.26, 782.2, 782.82, 783.89, 782.77, 780.04, 776.07, 773.18, 772.29, 772.7, 771.59, 770.13, 770.04, 770.21, 770.05, 768.54, 764.87, 762.48, 762.1, 761.96, 760.79, 758.73, 757.38, 756.99, 756.64, 757.13, 756.69, 755.5, 753.96, 752.37, 750.18, 748.1, 745.96, 745.17, 746.58, 747.59, 748.07, 748.15, 748.15, 748.18, 747.57, 746.27, 745.9, 745.61, 744.81, 743.17, 740.22, 737.96, 736.58, 736.03, 735.75, 734.99, 731.81, 731.01, 731.57, 733.07, 734.09, 734.62, 734.75, 732.78, 731.38, 730.39, 728.92, 727.3, 725.68, 723.41, 721.56, 720.35, 720.32, 721.18, 721.68, 723.09, 724.08, 724.57, 724.44, 722.92, 721.2, 719.36, 717.74, 716.27, 714.54, 712.79, 710.78, 708.63, 706.5, 704.85, 703.65, 703.14, 702.72, 703.18, 705.02, 705.57, 705.36, 705.28, 705.09, 704.1, 703.96, 704.23, 704.43, 703.8, 702.82, 700.81, 697.46, 694.0, 689.65, 688.64, 688.84, 688.88, 687.71, 688.97, 688.8, 687.46, 686.21, 685.44, 683.76, 683.0, 682.33, 679.92, 679.55, 677.65, 675.34, 676.07, 677.19, 677.84, 676.8, 674.38, 672.07, 671.1, 670.78, 670.74, 670.11, 668.27, 665.77, 663.55, 662.55, 662.07, 661.17, 660.17, 659.42, 658.82, 658.51, 658.2, 657.89, 657.51, 656.68, 655.82, 654.85, 654.16, 653.38, 652.65, 652.08, 651.39, 650.43, 650.29, 650.05, 649.83, 650.07, 649.97, 649.42, 648.11, 646.43, 644.41, 642.74, 640.73, 639.78, 639.71, 639.44, 637.51, 636.3, 636.47, 635.39, 632.44, 629.05, 629.35, 629.45, 629.39, 631.75, 630.52, 628.02, 627.6, 628.76, 629.05, 630.24, 629.09, 625.97, 624.19, 624.51, 622.97, 623.97, 621.87, 617.97, 616.47, 616.63, 615.36, 615.72, 613.93, 610.82, 608.92, 609.78, 608.09, 608.21, 606.64, 600.28, 600.05, 599.02, 598.7, 598.9, 598.43, 598.25, 598.23, 599.3, 599.58, 598.91, 597.85, 596.45, 593.81, 590.52, 587.91, 587.38, 587.73, 587.8, 588.41, 589.22, 589.98, 590.67, 590.83, 590.8, 589.05, 586.52, 584.72, 583.51, 582.52, 581.49, 580.42, 579.3, 578.5, 578.28, 577.97, 577.42, 577.25, 577.23, 577.04, 576.58, 575.52, 574.15, 572.24, 569.9, 567.76, 566.0, 563.57, 561.67, 560.72, 562.04, 562.75, 563.84, 564.54, 564.87, 564.87, 564.37, 563.42, 563.15, 562.67, 562.21, 562.27, 559.26, 556.25, 553.53, 550.41, 547.59, 546.15, 544.52, 542.12, 540.03, 539.58, 537.15, 534.4, 532.14, 531.09, 531.33, 531.96, 533.54, 534.12, 533.88, 534.56, 534.02, 532.92, 530.53, 528.36, 527.54, 526.61, 525.38, 525.85, 525.72, 525.3, 524.67, 524.26, 522.99, 521.89, 521.24, 520.03, 519.07, 517.98, 516.12, 514.85, 513.47, 512.38, 511.65, 510.42, 508.26, 504.39, 501.04, 500.31, 500.02, 500.29, 501.08, 500.51, 499.1, 497.27, 495.03, 491.4, 488.36, 486.09, 484.4, 483.54, 483.06, 482.21, 481.97, 482.59, 482.85, 484.26, 485.89, 486.27, 484.93, 484.56, 484.12, 483.29, 483.62, 484.18, 484.65, 484.25, 483.41, 481.51, 479.0, 475.94, 473.24, 470.6, 469.54, 469.87, 469.18, 467.91, 467.41, 467.29, 467.44, 465.99, 464.75, 463.3, 462.03, 460.98, 461.3, 461.87, 460.48, 460.22, 461.51, 462.97, 462.51, 461.51, 458.91, 457.58, 457.81, 458.01, 457.26, 457.34, 457.58, 456.59, 454.5, 452.46, 451.73, 450.18, 448.18, 447.65, 447.0, 446.39, 444.73, 443.56, 444.06, 442.93, 441.84, 443.28, 442.92, 442.54, 440.49, 440.51, 441.56, 440.24, 439.94, 440.47, 438.73, 435.99, 433.86, 432.57, 432.19, 430.23, 429.64, 428.58, 427.39, 425.03, 423.13, 422.68, 423.17, 422.14, 422.16, 423.46, 423.65, 422.49, 421.36, 422.4, 422.31, 423.02, 424.2, 423.3, 420.94, 418.77, 417.0, 414.93, 414.37, 413.83, 412.44, 410.7, 410.42, 409.1, 408.67, 408.1, 406.63, 404.8, 403.27, 401.07, 399.7, 398.35, 397.03, 395.77, 394.71, 393.96, 393.51, 392.9, 392.29, 391.78, 391.2, 390.57, 389.94, 389.46, 388.99, 388.62};
float acceleration_raw[] = {31.9, 31.92, 31.92, 31.91, 31.92, 31.9, 31.9, 31.9, 31.91, 31.91, 31.91, 31.92, 31.91, 31.94, 31.94, 31.94, 31.95, 31.94, 31.93, 31.91, 31.91, 31.9, 31.88, 31.9, 31.92, 31.92, 31.91, 31.93, 31.93, 31.93, 31.93, 31.93, 31.92, 31.92, 31.92, 31.92, 31.93, 31.95, 31.92, 31.93, 31.95, 32.0, 32.6, 32.62, 84.39, 109.34, 125.04, 131.61, 134.22, 133.7, 133.02, 132.77, 132.53, 132.07, 131.64, 131.26, 130.82, 130.34, 130.11, 130.12, 130.33, 130.34, 130.68, 131.09, 131.41, 131.8, 132.3, 132.7, 132.93, 132.91, 132.51, 132.16, 131.58, 131.13, 130.62, 130.29, 130.0, 129.76, 129.8, 130.27, 130.75, 132.04, 133.33, 133.58, 132.48, 131.32, 130.52, 129.93, 129.51, 129.05, 128.67, 115.73, 76.58, 41.31, 32.36, 33.18, 31.36, 29.09, 28.35, 28.41, 28.89, 29.75, 30.06, 30.86, 33.21, 33.68, 32.98, 33.44, 34.96, 34.16, 34.19, 33.84, 32.41, 32.45, 31.31, 30.61, 31.5, 30.64, 31.26, 29.96, 31.8, 32.84, 32.15, 31.73, 31.13, 30.09, 29.65, 29.35, 29.22, 28.57, 28.69, 28.59, 29.08, 29.42, 29.19, 28.82, 28.15, 27.25, 26.89, 26.55, 26.69, 26.67, 26.26, 26.29, 25.54, 25.89, 25.6, 25.35, 25.49, 25.25, 24.68, 24.29, 24.43, 24.4, 24.42, 23.93, 23.48, 23.44, 23.49, 24.09, 23.93, 23.77, 23.29, 22.35, 22.24, 22.48, 21.78, 21.87, 21.67, 21.95, 21.98, 21.72, 21.87, 21.1, 21.33, 21.16, 20.98, 20.7, 20.64, 20.31, 19.98, 19.6, 19.83, 19.45, 19.5, 19.49, 19.34, 19.31, 19.35, 19.1, 19.15, 19.22, 18.99, 18.93, 18.65, 18.52, 18.38, 18.3, 18.5, 18.58, 19.2, 20.34, 20.93, 21.59, 21.54, 20.58, 19.57, 20.45, 18.97, 16.32, 20.48, 22.35, 23.48, 19.14, 17.18, 19.26, 16.39, 18.86, 17.03, 19.63, 20.44, 21.47, 21.48, 21.59, 21.29, 18.26, 20.06, 17.75, 17.54, 16.25, 18.38, 18.08, 17.3, 19.5, 19.52, 17.99, 18.19, 16.91, 16.2, 17.83, 18.46, 16.51, 17.82, 17.51, 18.23, 18.57, 18.01, 17.97, 17.01, 17.51, 18.0, 17.78, 16.5, 17.04, 17.02, 16.5, 17.18, 17.65, 17.22, 16.97, 17.26, 16.76, 16.93, 16.98, 15.64, 15.25, 15.67, 15.84, 16.42, 16.02, 16.35, 16.39, 15.91, 15.65, 15.24, 15.3, 15.38, 14.91, 14.77, 14.37, 14.31, 14.54, 13.93, 14.86, 14.69, 13.56, 13.68, 13.84, 14.06, 14.4, 14.09, 13.61, 13.27, 12.88, 13.08, 13.28, 13.15, 12.89, 13.1, 12.81, 12.83, 12.31, 12.29, 12.52, 12.59, 12.64, 11.86, 11.97, 11.32, 11.45, 11.34, 11.3, 11.2, 11.58, 11.7, 11.22, 11.11, 11.0, 11.14, 11.37, 10.8, 10.89, 10.6, 10.5, 10.69, 10.47, 10.56, 10.33, 10.12, 10.13, 9.88, 9.67, 10.12, 9.59, 9.7, 9.44, 9.05, 9.3, 8.94, 9.31, 9.15, 9.3, 8.91, 9.04, 9.24, 8.8, 8.49, 8.25, 8.88, 8.62, 8.49, 8.77, 8.44, 8.52, 7.94, 8.2, 7.9, 7.98, 7.94, 7.79, 7.59, 7.7, 7.67, 7.85, 7.68, 7.86, 7.59, 7.46, 7.4, 7.37, 6.99, 6.71, 7.5, 7.54, 7.3, 7.0, 6.77, 6.71, 6.33, 6.66, 6.48, 6.34, 6.28, 6.33, 6.18, 6.33, 6.25, 6.33, 6.67, 6.39, 6.12, 6.19, 6.0, 5.93, 5.76, 5.94, 5.64, 5.52, 5.66, 5.45, 5.47, 5.3, 5.45, 5.46, 5.25, 5.12, 5.06, 5.23, 5.32, 5.41, 5.09, 4.92, 4.97, 4.85, 4.66, 4.79, 4.67, 4.63, 4.83, 4.67, 4.51, 4.53, 4.54, 4.4, 4.47, 4.34, 4.34, 4.5, 4.51, 4.5, 4.46, 4.24, 4.37, 4.43, 4.26, 4.13, 4.22, 3.97, 4.13, 4.3, 4.33, 4.14, 4.05, 3.92, 3.93, 4.0, 3.89, 3.79, 3.76, 3.77, 3.77, 3.66, 3.56, 3.64, 3.74, 3.67, 3.65, 3.77, 3.76, 3.57, 3.55, 3.53, 3.61, 3.7, 3.5, 3.4, 3.27, 3.33, 3.29, 3.2, 3.19, 3.23, 3.24, 3.1, 3.09, 3.17, 3.2, 3.05, 3.07, 3.16, 3.1, 2.99, 3.05, 3.02, 2.97, 2.9, 2.93, 2.93, 2.89, 2.84, 2.86, 2.91, 2.79, 2.79, 2.82, 2.71, 2.67, 2.76, 2.68, 2.74, 2.84, 2.76, 2.71, 2.68, 2.69, 2.68, 2.71, 2.79, 2.67, 2.72, 2.67, 2.74, 2.74, 2.66, 2.75, 2.6, 2.56, 2.6, 2.73, 2.67, 2.47, 2.52, 2.5, 2.52, 2.49, 2.41, 2.37, 2.44, 2.37, 2.31, 2.26, 2.21, 2.26, 2.35, 2.35, 2.24, 2.25, 2.24, 2.27, 2.32, 2.29, 2.32, 51.27, 37.95, 27.49, 16.26, 11.69, 7.71, 11.17, 26.85, 19.27, 11.64, 9.57, 8.74, 5.85, 4.41, 9.47, 14.46, 17.01, 20.44, 14.74, 40.94, 85.31, 104.0, 78.74, 37.87, 23.55, 16.44, 16.18, 16.66, 16.78, 18.64, 20.25, 22.92, 23.67, 20.86, 19.25, 19.07, 19.13, 16.07, 16.45, 16.35, 18.26, 15.04, 15.18, 22.91, 24.23, 25.48, 27.4, 27.19, 30.9, 22.56, 21.6, 22.42, 25.38, 25.32, 25.25, 32.29, 28.23, 27.42, 21.15, 41.66, 53.97, 58.86, 76.25, 53.74, 41.42, 47.48, 52.71, 63.67, 75.92, 73.42, 62.19, 53.17, 66.53, 80.6, 84.74, 81.87, 71.13, 57.72, 48.13, 39.72, 30.46, 25.99, 26.98, 26.99, 26.65, 28.81, 35.27, 42.58, 49.95, 65.82, 94.74, 96.89, 82.86, 79.98, 115.83, 117.25, 123.53, 108.28, 94.12, 81.57, 76.38, 68.12, 59.64, 64.37, 77.78, 98.39, 113.81, 111.75, 96.46, 58.22, 45.76, 38.56, 39.64, 35.33, 41.7, 55.23, 48.32, 48.85, 51.74, 49.02, 43.43, 32.34, 29.47, 28.4, 29.99, 32.35, 39.3, 43.04, 55.7, 61.59, 73.63, 63.85, 56.35, 34.06, 19.12, 19.27, 26.06, 30.38, 27.01, 26.87, 26.22, 20.38, 15.55, 11.71, 18.02, 27.28, 31.77, 44.49, 50.66, 55.8, 49.62, 52.8, 46.06, 40.17, 28.06, 25.37, 23.27, 18.24, 28.05, 30.97, 34.6, 39.18, 46.0, 62.55, 49.48, 45.49, 43.6, 39.71, 33.74, 24.39, 22.0, 16.3, 11.93, 13.14, 16.19, 19.71, 22.55, 26.71, 32.13, 32.86, 35.85, 37.71, 38.67, 35.3, 31.86, 30.55, 23.1, 17.8, 15.18, 14.75, 19.43, 26.12, 29.15, 32.38, 39.67, 40.41, 39.97, 40.81, 38.02, 35.55, 32.88, 26.27, 27.14, 28.57, 26.34, 27.19, 24.61, 23.54, 19.57, 15.63, 14.37, 14.45, 14.03, 14.22, 13.98, 15.69, 16.72, 19.04, 27.38, 26.9, 24.68, 24.3, 24.67, 25.36, 26.3, 28.08, 23.77, 21.34, 21.92, 19.77, 19.88, 19.43, 22.21, 25.73, 27.04, 25.22, 23.91, 22.18, 21.62, 26.31, 28.99, 39.25, 39.04, 34.28, 37.97, 30.45, 25.27, 26.84, 16.17, 18.97, 23.49, 46.55, 30.7, 24.4, 25.36, 30.51, 36.05, 36.63, 45.42, 69.7, 102.58, 66.71, 84.53, 99.75, 119.8, 122.13, 127.9, 114.87, 123.65, 112.17, 101.49, 99.4, 104.67, 94.23, 64.86, 46.33, 53.69, 74.19, 83.99, 99.67, 103.53, 96.95, 87.09, 105.06, 84.44, 51.94, 43.98, 32.9, 23.22, 19.54, 23.77, 39.63, 49.29, 41.96, 48.03, 49.61, 38.42, 41.4, 54.23, 88.38, 105.86, 114.8, 101.1, 83.76, 77.11, 60.77, 35.96, 23.11, 15.91, 13.81, 14.16, 15.38, 16.78, 21.86, 26.04, 19.3, 11.73, 9.49, 11.08, 11.0, 14.15, 21.75, 28.61, 32.72, 42.19, 48.14, 45.6, 44.45, 51.29, 54.38, 50.8, 32.99, 40.51, 42.01, 50.85, 49.3, 40.11, 48.8, 33.44, 27.54, 29.72, 33.95, 28.4, 33.45, 32.88, 23.63, 36.21, 31.2, 28.54, 30.62, 38.42, 35.97, 32.19, 35.61, 41.34, 43.67, 33.08, 30.81, 33.67, 33.31, 31.23, 27.32, 22.65, 22.33, 29.36, 29.18, 27.26, 26.66, 28.18, 26.21, 26.99, 25.05, 29.02, 40.82, 37.34, 32.92, 32.07, 30.94, 27.09, 26.02, 25.45, 24.5, 25.12, 28.23, 23.91, 22.24, 18.62, 21.0, 24.96, 26.82, 28.19, 43.77, 39.35, 39.41, 36.26, 37.09, 38.89, 34.17, 46.94, 50.94, 49.69, 71.72, 61.81, 48.79, 57.45, 49.2, 40.63, 43.21, 37.05, 41.75, 44.57, 46.09, 41.5, 32.52, 25.08, 20.35, 24.04, 25.08, 23.02, 33.45, 37.16, 32.96, 28.55, 34.26, 67.27, 79.61, 81.39, 97.97, 92.34, 82.11, 67.08, 74.06, 63.18, 55.94, 51.96, 48.29, 37.36, 33.38, 17.73, 20.89, 31.24, 36.31, 35.93, 35.13, 25.71, 14.32, 16.08, 25.48, 39.58, 59.94, 73.34, 80.32, 81.3, 80.82, 64.97, 48.2, 45.64, 51.3, 42.23, 50.77, 53.79, 55.35, 52.39, 40.29, 38.29, 39.29, 35.19, 38.44, 40.68, 31.09, 27.49, 23.76, 24.93, 19.41, 14.63, 11.5, 11.14, 9.79, 11.29, 13.71, 21.71, 24.97, 29.92, 28.37, 30.14, 36.46, 40.67, 44.96, 56.74, 56.87, 54.38, 46.86, 42.58, 37.87, 36.54, 41.14, 38.22, 40.94, 44.2, 46.24, 41.99, 40.92, 39.22, 39.82, 30.74, 29.86, 38.13, 34.87, 29.83, 26.64, 24.42, 24.17, 27.56, 26.63, 22.87, 23.03, 19.68, 32.23, 23.66, 18.22, 16.12, 18.04, 17.5, 16.73, 15.64, 15.86, 16.25, 16.4, 16.78, 15.69, 26.05, 18.69, 12.63, 8.44, 5.66, 5.59, 14.02, 11.93, 14.72, 18.13, 22.94, 20.03, 22.44, 28.83, 26.31, 27.21, 24.3, 24.89, 27.97, 33.69, 39.98, 42.15, 32.32, 31.15, 35.26, 52.25, 51.43, 43.25, 31.71, 33.36, 35.78, 38.91, 39.83, 50.42, 45.85, 50.24, 45.0, 45.16, 51.19, 65.81, 76.83, 97.74, 95.6, 85.49, 73.15, 56.49, 64.59, 45.31, 33.5, 33.0, 30.21, 24.5, 23.66, 15.32, 14.57, 19.68, 69.62, 96.33, 78.91, 47.65, 33.44, 37.57, 35.37, 42.44, 72.42, 72.36, 65.9, 61.16, 60.14, 59.32, 67.42, 58.99, 44.23, 55.29, 39.35, 36.42, 40.52, 44.85, 47.01, 40.89, 54.0, 68.79, 53.46, 40.96, 27.47, 44.3, 34.16, 34.47, 42.97, 33.85, 29.18, 27.33, 25.48, 22.16, 19.8, 19.61, 26.05, 38.13, 56.9, 76.29, 70.08, 93.87, 90.8, 80.77, 58.84, 58.48, 43.23, 42.78, 35.93, 32.04, 27.57, 21.82, 18.76, 14.61, 13.41, 11.74, 10.92, 10.62, 11.33, 11.25, 11.36, 12.82, 11.73, 11.14, 11.33, 14.45, 22.22, 126.72, 82.9, 60.88, 50.79, 110.26, 98.31, 100.16, 98.13, 90.7, 80.55, 69.06, 60.42, 52.24, 43.99, 58.62, 53.86, 61.0, 57.48, 60.54, 75.46, 75.37, 61.78, 60.94, 53.32, 68.04, 63.18, 64.75, 56.94, 53.45, 45.87, 40.14, 32.9, 29.61, 63.54, 91.44, 70.57, 48.18, 41.92, 37.14, 32.64, 41.12, 39.95, 36.35, 38.7, 37.22, 35.52, 41.09, 49.35, 39.33, 37.06, 37.6, 36.36, 44.75, 39.45, 35.37, 34.01, 30.27, 29.59, 28.31, 26.96, 27.46, 23.45, 24.09, 24.61, 26.7, 20.74, 19.82, 19.92, 23.2, 26.1, 29.19, 34.03, 40.25, 53.46, 57.54, 52.75, 46.06, 42.03, 36.07, 32.78, 30.36, 29.4, 30.82, 34.23, 33.42, 39.38, 40.62, 45.53, 45.87, 52.44, 56.29, 59.34, 59.1, 48.36, 34.9, 36.77, 42.5, 32.9, 39.54, 41.43, 42.42, 33.47, 37.78, 34.73, 25.23, 26.69, 26.03, 33.81, 44.58, 50.22, 56.71, 55.06, 56.59, 59.83, 57.61, 53.4, 52.98, 43.88, 31.43, 28.57, 33.25, 51.29, 60.64, 64.98, 65.79, 69.44, 46.0, 31.33, 30.03, 22.16, 13.35, 17.94, 25.45, 36.58, 50.44, 48.56, 51.3, 47.16, 39.58, 36.7, 31.21, 24.46, 23.4, 23.31, 16.76, 19.25, 25.3, 27.22, 33.8, 38.63, 31.7, 27.54, 41.06, 41.14, 54.38, 40.51, 34.72, 30.52, 27.58, 36.06, 29.69, 24.32, 29.17, 28.12, 32.69, 29.52, 30.58, 24.42, 20.0, 16.1, 13.2, 13.3, 16.08, 20.25, 26.5, 30.12, 35.2, 39.27, 43.27, 58.63, 81.57, 58.02, 44.85, 44.51, 47.21, 46.8, 36.93, 45.35, 45.92, 45.93, 67.19, 49.43, 76.11, 105.91, 73.2, 57.51, 60.64, 66.47, 92.11, 66.61, 62.79, 47.46, 38.28, 36.37, 48.72, 46.17, 42.1, 43.69, 22.96, 15.98, 15.5, 18.22, 22.57, 22.03, 27.63, 34.58, 40.95, 53.53, 74.42, 87.63, 105.05, 140.33, 86.83, 69.87, 42.62, 55.35, 47.21, 37.71, 34.02, 24.89, 21.07, 22.72, 23.31, 27.45, 31.22, 25.65, 26.53, 22.07, 22.21, 21.88, 16.13, 15.18, 16.88, 17.32, 23.77, 31.11, 29.53, 33.95, 33.22, 31.44, 87.54, 87.12, 67.3, 56.55, 67.02, 62.54, 67.36, 67.41, 40.27, 50.0, 49.14, 43.4, 35.65, 32.16, 24.59, 21.64, 20.84, 24.89, 41.45, 36.08, 94.87, 77.05, 74.99, 52.09, 37.99, 33.03, 31.94, 49.73, 49.0, 51.04, 51.89, 46.92, 52.15, 59.67, 73.95, 58.66, 59.47, 49.34, 43.55, 40.66, 41.29, 43.76, 38.96, 40.09, 36.86, 32.7, 32.36, 31.92, 30.93, 30.17, 29.11, 27.42, 27.39, 28.6, 25.78, 23.24, 23.03, 24.51, 24.31, 23.35, 22.71, 22.75, 21.77, 20.16, 19.91, 21.82, 25.24, 25.8, 24.94, 23.52, 22.95, 23.07, 23.45, 23.25, 28.41, 24.17, 28.07, 29.76, 30.06, 87.59, 63.27, 50.03, 53.1, 51.2, 48.14, 40.68, 34.94, 27.12, 46.61, 49.53, 51.86, 56.19, 57.77, 82.18, 41.26, 27.63, 22.09, 23.42, 26.62, 28.78, 37.08, 46.31, 47.78, 49.27, 47.09, 46.39, 46.68, 38.73, 34.44, 33.91, 32.95, 29.12, 21.23, 29.35, 37.98, 31.93, 31.22, 30.73, 30.19, 33.23, 34.71, 32.86, 25.41, 33.01, 32.02, 35.21, 37.03, 49.1, 48.08, 48.53, 51.33, 39.18, 42.29, 39.01, 32.64, 35.68, 43.29, 50.13, 50.02, 52.08, 50.15, 46.77, 33.81, 33.98, 26.53, 30.5, 39.74, 55.82, 75.95, 93.97, 86.25, 72.04, 50.28, 39.38, 41.35, 48.18, 63.61, 78.07, 95.54, 87.13, 66.36, 60.58, 52.11, 48.32, 40.23, 57.71, 43.91, 90.69, 68.86, 51.58, 41.8, 35.69, 32.31, 28.42, 23.43, 22.27, 26.01, 32.4, 37.29, 38.51, 37.08, 36.69, 27.55, 25.12, 30.26, 41.73, 59.78, 56.13, 53.16, 54.66, 85.24, 102.29, 126.99, 103.29, 68.91, 48.04, 51.23, 51.24, 47.11, 50.3, 66.51, 60.43, 55.0, 55.45, 49.3, 38.85, 31.87, 26.45, 22.61, 21.35, 19.34, 19.17, 22.06, 25.49, 28.39, 30.01, 31.03, 32.58, 35.34, 38.83, 35.87, 32.29, 32.35, 33.22, 25.33, 35.57, 38.13, 41.63, 33.19, 23.37, 29.36, 25.22, 22.89, 23.05, 22.18, 21.61, 21.19, 19.63, 18.28, 16.9, 16.2, 18.45, 23.17, 19.5, 17.78, 29.38, 47.2, 47.83, 52.5, 68.59, 103.86, 111.8, 80.39, 77.69, 71.75, 73.86, 94.99, 86.9, 88.38, 76.05, 74.15, 65.17, 57.75, 64.61, 66.44, 70.27, 76.37, 72.62, 75.87, 78.81, 90.19, 92.97, 99.87, 94.11, 83.55, 75.25, 62.76, 46.77, 56.34, 33.47, 22.75, 19.17, 21.68, 22.93, 23.05, 25.66, 26.92, 25.99, 25.47, 24.44, 22.7, 20.55, 18.38, 15.94, 14.46, 14.11, 13.42, 13.73, 13.37, 12.92, 13.4, 14.19, 53.04, 48.38, 26.1, 22.6, 37.79, 58.19, 72.53, 50.19, 38.03, 32.95, 36.83, 49.96, 61.61, 91.89, 112.83, 133.51, 116.39, 107.76, 100.09, 116.96, 141.83, 135.13, 128.33, 102.74, 90.61, 81.87, 111.44, 117.86, 118.25, 104.49, 103.81, 116.09, 137.91, 135.68, 127.37, 107.33, 121.45, 142.83, 145.25, 133.96, 93.48, 91.69, 94.18, 115.16, 127.18, 129.69, 117.08, 100.99, 106.68, 129.41, 128.88, 115.42, 88.86, 71.19, 66.87, 70.74, 71.47, 69.88, 63.38, 55.75, 42.43, 34.33, 35.63, 27.95, 26.88, 26.03, 26.7, 23.64, 21.64, 18.66, 16.97, 16.16, 18.46, 20.13, 23.54, 23.33, 25.26, 30.32, 41.01, 31.94, 31.91, 32.84, 35.39, 29.98, 36.11, 31.27, 28.2, 33.54, 33.86, 30.33, 25.2, 24.4, 22.39, 19.84, 21.41, 23.03, 22.33, 22.82, 17.74, 17.45, 15.55, 13.42, 13.67, 18.39, 18.46, 18.03, 23.36, 36.17, 45.79, 80.85, 105.67, 126.62, 133.86, 128.87, 131.71, 128.21, 115.88, 97.11, 75.77, 57.98, 49.32, 34.68, 32.6, 34.9, 38.51, 49.65, 76.73, 117.22, 73.65, 70.97, 76.22, 78.44, 81.3, 70.7, 61.39, 57.69, 54.22, 50.38, 50.55, 44.67, 36.29, 34.75, 37.21, 33.77, 29.13, 30.84, 26.73, 26.03, 34.46, 35.92, 38.35, 69.17, 78.96, 113.89, 96.94, 91.16, 87.59, 135.76, 150.05, 138.86, 120.12, 93.19, 71.37, 53.27, 44.32, 33.53, 21.8, 24.12, 24.46, 25.61, 31.62, 38.74, 48.39, 62.38, 74.19, 91.02, 96.46, 104.2, 103.33, 103.96, 90.2, 91.7, 77.63, 62.55, 52.34, 48.55, 41.28, 36.07, 30.57, 28.37, 24.72, 26.83, 40.67, 40.18, 47.48, 54.63, 45.28, 38.61, 23.82, 21.49, 32.41, 23.52, 52.8, 34.27, 61.84, 80.96, 91.38, 82.48, 83.49, 65.98, 47.87, 24.37, 30.55, 49.55, 71.67, 93.67, 97.17, 92.13, 77.83, 62.27, 42.42, 45.95, 55.81, 38.19, 58.78, 50.84, 67.39, 51.11, 37.72, 52.87, 67.0, 66.54, 73.98, 74.02, 48.3, 54.94, 54.82, 79.36, 48.48, 51.93, 73.65, 76.93, 69.79, 70.88, 83.43, 81.68, 72.99, 90.76, 81.62, 75.11, 67.46, 76.59, 77.37, 90.09, 90.98, 108.03, 98.57, 76.84, 53.95, 62.23, 86.48, 105.87, 103.97, 105.25, 91.94, 82.23, 106.7, 124.97, 137.32, 130.4, 126.38, 123.15, 115.66, 106.63, 112.95, 152.38, 153.02, 135.08, 123.59, 155.0, 177.71, 168.16, 128.21, 135.56, 168.09, 177.42, 139.1, 119.83, 128.07, 132.87, 124.68, 104.94, 72.95, 60.44, 70.37, 86.83, 81.21, 66.6, 48.54};


// Updates the global measurement variables from the m_simSensorValues hashmap, as opposed to trying to poll sensors
String getSimulatedMeasurements() {

    FilterData(altitude_raw[i] * METERS_TO_FEET, acceleration_raw[i]); 

    // print measurements
    String movementData = String(m_bmp.pressure/100.0) + "," + 
                           String(m_bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA)) + "," + 
                           String(accel.acceleration.x*METERS_TO_FEET) + "," + 
                           String(accel.acceleration.y*METERS_TO_FEET) + "," + 
                           String(accel.acceleration.z*METERS_TO_FEET) + "," + 
                           String(gyro.gyro.x) + "," + 
                           String(gyro.gyro.y) + "," + 
                           String(gyro.gyro.z) + "," + 
                           String(gAltFiltered) + "," + 
                           String(gVelocityFiltered) + "," + 
                           String(gAccelFiltered);
    String timeData = String(millis() - gStartTime);
    String sensorData = String(ReadThermometer());
    // if (DEBUG) {Serial.println(timeData + "," + movementData + "," + sensorData + "," + String(gAtsPosition));}

    if (j < 200) j++;
    else if (i < 2050) {
        i++;
    }

    return timeData + "," + movementData + "," + sensorData + "," + String(gAtsPosition);

    // collectSimulatedData();
        
        // float altitude_raw = dataAsFloat("altitude_raw");
        // float acceleration_raw = dataAsFloat("acceleration_raw");

        // FilterData(altitude_raw, acceleration_raw);

        // String movementData = String(altitude_raw) + "," + String(acceleration_raw) + "," + String(gAltFiltered) + "," + String(gVelocityFiltered) + "," + String(gAccelFiltered);

        // String timeData = String(millis() - gStartTime);

        // String sensorData = String(dataAsFloat("temperature")); // IK its a little redundant but it's for consistency

        // return timeData + "," + movementData + "," + sensorData + "," + gAtsPosition;
}
#endif
