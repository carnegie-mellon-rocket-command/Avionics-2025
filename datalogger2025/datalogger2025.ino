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
const bool DEBUG = true;

// How frequently data should be collected (in milliseconds)
const int LOOP_TARGET_MS = 30;

// Target altitude in feet
#if SUBSCALE
    const float ALT_TARGET = 2000.0f; // ft
#else
    const float ALT_TARGET = 4500.0f; // ft
#endif

// Acceleration threshold for launch detection (ft/s^2)
const float ACCEL_THRESHOLD = 2*GRAVITY;
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
float gAltFiltered, gVelocityFiltered, gAccelFiltered;
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
    #if SIMULATE
        startSimulation();
    #endif

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
            WriteData("time, pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, gAltFiltered (ft), gVelocityFiltered (ft/s), gAccelFiltered (ft/s^2), temperature (from IMU, degrees C), gAtsPosition (servo degrees)\n");

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
    if (DEBUG) {Serial.println(timeData + "," + movementData + "," + sensorData + "," + String(gAtsPosition));}
    return timeData + "," + movementData + "," + sensorData + "," + String(gAtsPosition);
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
    Serial << gAccelFiltered << "\n";
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
    #if SIMULATE
        return;
    #endif
    #if SKIP_ATS
        return;
    #endif
    m_atsServo.attach(ATS_PIN);
}
/** @brief turn off ATS servo
 * (for saving power) */
void detachATS() {
    requires(m_atsServo.attached());
    #if SIMULATE
        return;
    #endif
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
    #if SIMULATE
        return;
    #endif
    #if SKIP_ATS
        return;
    #endif
    float pos = (ATS_MAX - ATS_MIN) * (1.0f - percent_rot) + ATS_MIN;
    m_atsServo.write(int(pos));
    if (DEBUG) {Serial.println("ATS position set to " + String(pos));}
}


/** @brief adjust ATS with PID
 * Adjust the ATS based on the current altitude and desired apogee
 */
void AdjustATS() {
    float targetAcceleration = abs(pow(gVelocityFiltered,2)/(2*(ALT_TARGET-gAltFiltered)))
     // Retract ATS fully after 18 seconds
    if (millis() - gLaunchTime > 18000) {
        setATSPosition(ATS_IN);
        return;
    }
    // Fully deploy ATS if reached Altitude target
    if (gAltFiltered >= ALT_TARGET) {
        gAtsPosition = ATS_OUT;
    }
    else {
        // Calculate desired surface-area to reach target altitude
        float target_area = (pow(gVelocityFiltered, 2)/(ALT_TARGET - gAltFiltered) - 2*GRAVITY)*ROCKET_MASS/(gVelocityFiltered*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT);
        Serial.println("Old ATS pos: " + String(target_area/ATS_MAX_SURFACE_AREA));
        // Calculate error in acceleration
        float error = gAccelFiltered - targetAcceleration; // positive if drag + gravity >= target, means if <, we are going TOO FAST and need to slow down
        // calculate adjustment
        float adjustment;
        if (error > 0){ //too slow
            adjustment = 0;
        } 
        else { //too flast
            adjustment = PIDFactor(abs(error), 0.03,0); // should normalize to 0 to 1
        }
        gAtsPosition = adjustment;
    }

    // ATS window
    if (millis() - gLaunchTime > 4500) {
        // Adjust ATS based on position
        setATSPosition(gAtsPosition);
        Serial.println("ATS position: " + String(gAtsPosition));
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

// Begin the simulation by waiting for the host program to start and give confirmation over serial
void startSimulation() {
    m_simSensorValues->insert("altitude_raw", "0.0");
    m_simSensorValues->insert("acceleration_raw", "0.0");
    m_simSensorValues->insert("temperature", "0.0");

    // Await connection from host computer
    while (!Serial.readString()) {
        delay(10);
    }

    Serial.println("(simulation) Arduino is ready!");
}

// Reads the serial buffer for the most recent data sent by the hosts, and updates the m_simSensorValues hashmap accordingly
void collectSimulatedData() {
    String serialBuffer = Serial.readString();
    String currentKey, currentValue = "";
    for (int i = 0; i < serialBuffer.length(); i++) {
        if (serialBuffer[i] == ':') {
            currentKey = currentValue;
            currentValue = "";
        } else if (serialBuffer[i] == ',') {
            m_simSensorValues->insert(currentKey, currentValue);
            Serial.println("Received "+currentKey+", "+currentValue);
            currentKey = "";
            currentValue = "";
        } else {
            currentValue += serialBuffer[i];
        }
    }
}

float dataAsFloat(String measurement_name) {
    return m_simSensorValues->search(measurement_name).toFloat();
}

// Updates the global measurement variables from the m_simSensorValues hashmap, as opposed to trying to poll sensors
String getSimulatedMeasurements() {
        collectSimulatedData();
        
        float altitude_raw = dataAsFloat("altitude_raw");
        float acceleration_raw = dataAsFloat("acceleration_raw");

        FilterData(altitude_raw, acceleration_raw);

        String movementData = String(altitude_raw) + "," + String(acceleration_raw) + "," + String(gAltFiltered) + "," + String(gVelocityFiltered) + "," + String(gAccelFiltered);

        String timeData = String(millis() - gStartTime);

        String sensorData = String(dataAsFloat("temperature")); // IK its a little redundant but it's for consistency

        return timeData + "," + movementData + "," + sensorData + "," + gAtsPosition;
}
#endif
