/*
Carnegie Schmellon Rocketry Club: Precision INstrumented Experimental Aerial Propulsion Payload 
                                  for Low-altitude Exploration ("PINEAPPLE") data logger             
(see what I did there lol)

Made by the 2025 Avionics team :D
*/

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
const float accel_threshold = 2*GRAVITY;
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

// MEASUREMENT VARIABLES

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

// Entry point to the program
// Initializes all sensors and devices, and briefly tests the ATS
void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");
    LEDSetup();

    #if SIMULATE
        startSimulation();
    #endif

    // Initialize SD card
    if (!initializeSDCard()) {
        Serial.println("SD card setup failed. Aborting.");
        LEDError();
    }

    // Initialize sensors
    if (!setupSensors()) {
        Serial.println("Sensor setup failed. Aborting.");
        LEDError();
    }
    
    // Test if all sensors on Altimeter works
    if (!TestSensors()){
        Serial.println("one or more altimeter sensors are not working");
        LEDError();
    }
    
    Serial.println("All sensors working!");
    testATS();

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
    start_time = millis();
}


// Repeats indefinitely after setup() is finished
/** @brief collect/log data  and control ATS */
void loop() {
    buffer = "";
    for (int i = 0; i < buffer_size; i++) {
        run_timer();  // ensures loop runs at a consistent rate
        // Get measurements from sensors and add to buffer
        buffer = buffer + getMeasurements() + "\n";
        // ****(Pre-Flight)****
        // Detect launch based on acceleration threshold
        if (acceleration_filtered > accel_threshold && !launched) {
            // Write CSV header to the file
            writeData("***************** START OF DATA ***************** TIME SINCE BOOT: " + String(millis()) + " ***************** TICK SPEED: " + String(loop_target) + "ms\n");
            writeData("time, pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, altitude_filtered (ft), velocity_filtered (ft/s), acceleration_filtered (ft/s^2), temperature (from IMU, degrees C), ats_position (servo degrees)\n");

            if (DEBUG) {Serial.println("Rocket has launched!");}
            launched = true;
            launch_time = millis();
            
            // Bring the ATS back online
            attachATS();
            setATSPosition(0.0);

            // Set status LED
            LEDLogging();
        }
        // ****(During Flight)****
        // If the rocket has launched, adjust the ATS as necessary, and detect whether the rocket has landed
        if (launched) {
            LEDFlying();
            adjustATS();
            if (detectLanding()) {landed = true;}
        }
    }
    
    if (launched) {writeData(buffer);}
    // ****(END OF FLIGHT)****
    if (landed) {
        // End the program
        detachATS();
        if (DEBUG) {Serial.println("Rocket has landed, ending program");}
        while (true);
    }
}


// Function to handle loop timing: delays the loop to ensure it runs at a consistent rate
void run_timer() {
    long temp_time = millis() - prev_loop_time;
    // Serial.println(temp_time);
    if (temp_time < loop_target) {
        delayMicroseconds((loop_target - temp_time) * 1000);
    }
    else if (temp_time > loop_target + 1) {
        Serial.println("Board is unable to keep up with target loop time of " + String(loop_target) + " ms (execution took "+ String(temp_time) + " ms)");
    }
    curr_time = millis();
    timer = curr_time - start_time;
    loop_time_delta = curr_time - prev_loop_time;
    // Serial.println(loop_time);
    prev_loop_time = curr_time;
}


// Initialize the SD card (returns true if successful, false if failed)
// Right now, tries to connect 10 times before going into an error state; could change so it keeps trying indefinitely
bool initializeSDCard() {
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


// Log data to the SD card in the file "datalog.txt"
void writeData(String text) {
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
/**  @brief Initialize all sensors and return true if successful, false if any have failed */
bool setupSensors() {
  #if SIMULATE
    Serial.println("(simulation) Sensors connected successfully!");
    return true;
  #endif

  if (setupLSM6DSOX() && setupBMP3XX()) {
    if (DEBUG) {Serial.println("Sensors initialized successfully!");}
    bmp.performReading();
    Serial.println("Setup reading fine");
    return true;
  }
  return false;
}

/**  @brief Setup Altimeter */
bool setupBMP3XX() {
  if (!bmp.begin_SPI(altimeter_chip_select)) {
    Serial.println("Unable to connect to altimeter");
    return false;
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("Altimeter good");
  bmp.performReading();
  Serial.print(bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA));
  return true;
}

/**  @brief Setup IMU */
bool setupLSM6DSOX() {
  if (!sox.begin_I2C()) {
    Serial.println("Unable to connect to IMU");
    return false;
  }
  return true;
}

/**  @brief Test Altimeter and IMU */
bool TestSensors() {
    if (!bmp.performReading()){
        Serial.println("one or more altimeter sensors are not working");
        return false;
    }
    if (!sox.getEvent(&accel, &gyro, &temp)){
        Serial.println("one or more IMU sensors are not working");
        return false;
    }
}

// ***************** Collecting/Logging Data *****************
/**  @brief  Read altitude from altimeter (in meters) and return it
* IMPORTANT: update altimeter with `performreading()` beforehand
* @returns raw altimeter reader 
*/
float readAltimeter() {
    float altimeter_data = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA) * METERS_TO_FEET;
    if (DEBUG) {Serial.println("Altimeter: " + String(altimeter_data));}
    return altimeter_data;
}

/** @brief Read IMU data, get vertical acceleration (in meters/second^2) 
 * IMPORTANT: update IMU with `getevent()` first for current reading
 * @returns acceleration (ft/s^2)
*/
float readIMU() {
    float imu_data = pow(pow(accel.acceleration.x,2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2), 0.5);
    // Convert to feet
    imu_data = imu_data * METERS_TO_FEET;
    if (DEBUG) {Serial.println("IMU: " + String(imu_data));}
    return imu_data;
}

/** @brief Read temperature 
 * IMPORTANT: update IMU with `getevent()` beforehand
 * from thermometer (on IMU in degrees C)
 */
float readThermometer() {
    float thermometer_data = temp.temperature;
    if (DEBUG) {Serial.println("Thermometer: " + String(thermometer_data));}
    return thermometer_data;
}

/** @brief refresh sensors and update globals with latest measurements
 * output string can be viewed when DEBUG is true
 * @returns a CSV string with format:
 * - time, pressure (hPa), altitude_raw (m), acceleration_raw_x (m/s^2), 
 * - acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, 
 * - altitude_filtered, velocity_filtered, acceleration_filtered, 
 * - temperature (from IMU, degrees C), ats_position (degrees)
 */
String getMeasurements() {
    #if SIMULATE
        // If simulation mode is ON, get simulated data from serial bus instead
        return getSimulatedMeasurements();
    #endif
    //update sensors
    bmp.performReading(); //update altimeter
    sox.getEvent(&accel, &gyro, &temp); //update imu
    
    filterData(readAltimeter(), readIMU()); //pass raw data to be filtered

    // print measurements
    String movement_data = String(bmp.pressure/100.0) + "," + 
                           String(bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA)) + "," + 
                           String(accel.acceleration.x*METERS_TO_FEET) + "," + 
                           String(accel.acceleration.y*METERS_TO_FEET) + "," + 
                           String(accel.acceleration.z*METERS_TO_FEET) + "," + 
                           String(gyro.gyro.x) + "," + 
                           String(gyro.gyro.y) + "," + 
                           String(gyro.gyro.z) + "," + 
                           String(altitude_filtered) + "," + 
                           String(velocity_filtered) + "," + 
                           String(acceleration_filtered);
    String time_data = String(millis() - start_time);
    String sensor_data = String(readThermometer());
    if (DEBUG) {Serial.println(time_data + "," + movement_data + "," + sensor_data + "," + String(ats_position));}
    return time_data + "," + movement_data + "," + sensor_data + "," + String(ats_position);
}

// Filter raw data and store it in global variables; still need to implement
void filterData(float alt, float acc) {
    float DT = ((float)loop_time_delta)/1000;

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
    altitude_filtered = KalmanFilter.x(0);
    velocity_filtered = KalmanFilter.x(1);
    acceleration_filtered = KalmanFilter.x(2);
    // Serial << alt << "," << acc <<"," << altitude_filtered << "," << velocity_filtered << "," << acceleration_filtered << "\n";
    Serial << acceleration_filtered << "\n";
    // Serial.println("Filtered Altitude: " + String(altitude_filtered) + " Filtered Velocity: " + String(velocity_filtered) + " Filtered Acceleration: " + String(acceleration_filtered));


    // NO FILTER vvvvvv
    // altitude_filtered = alt;
    // velocity_filtered = (altitude_filtered - previous_velocity_filtered) / (loop_time/1000);
    // acceleration_filtered = acc;


    // OLD FILTER vvvvvv

    // // Low pass filter: if data is sus, extrapolate from previous data instead
    // if (abs(alt - altitude_filtered) < 100) {
    //     alt = altitude_filtered + velocity_filtered * loop_time + 0.5 * acceleration_filtered * pow(loop_time, 2);
    // }
    // if (abs(acc - acceleration_filtered) < 100) {
    //     acc = acceleration_filtered;
    // }

    // // Kalman Filter

    // // Update altitude
    // // altitude_filtered_previous = altitude_filtered; // Should update via dynamics model (TODO)
    // altitude_filtered_previous = altitude_filtered + velocity_filtered * loop_time + 0.5 * acceleration_filtered * pow(loop_time, 2);
    // cov_altitude_previous = cov_altitude_current; // Should update via dynamics model (TODO)

    // altitude_filtered = altitude_filtered_previous + gain_altitude * (alt - altitude_filtered_previous);
    // cov_altitude_current = (1 - gain_altitude) * cov_altitude_previous;
    // gain_altitude = cov_altitude_current / (cov_altitude_current + variance_altitude);

    // // Update acceleration
    // acceleration_filtered_previous = acceleration_filtered; // Should update via dynamics model (TODO)
    // cov_acceleration_previous = cov_acceleration_current; // Should update via dynamics model (TODO)

    // acceleration_filtered = acceleration_filtered_previous + gain_acceleration * (acc - acceleration_filtered_previous);
    // cov_acceleration_current = (1 - gain_acceleration) * cov_acceleration_previous;
    // gain_acceleration = cov_acceleration_current / (cov_acceleration_current + variance_acceleration);

    // // Interpolate velocity
    // velocity_filtered = (altitude_filtered - altitude_filtered_previous) / loop_time;
}

/** @brief Detect if rocket has landed 
 * if the rocket has been reasonably still for 5 seconds, it is considered landed
*/
bool detectLanding() {
    if (launch_time != 0 && millis() - launch_time > 60000) {
        return true;
    }


    // if (velocity_filtered < velocity_threshold) {
    //     land_time = millis();
    // }
    // else {
    //     land_time = 0;
    // }

    // if (land_time != 0 && millis() - land_time > 5000) {
    //     return true;
    // }

    return false;
}
 

float pid_factor(int error, float Kp, float Kd)
{
  static int old_error = 0;

  float proportional = error * Kp;

  float derivative = (error - old_error) * Kd;
  old_error = error;

  return proportional + derivative; 
}

// ***************** ATS METHODS *****************

// Turns on the ATS servo
void attachATS() {
    #if SIMULATE
        return;
    #endif
    #if SKIP_ATS
        return;
    #endif

    ATS_servo.attach(ats_pin);
}
// Turns off the ATS servo (to save power)
void detachATS() {
    #if SIMULATE
        return;
    #endif
    #if SKIP_ATS
        return;
    #endif

    ATS_servo.detach();
}

// Sets how far the ATS is extended based on a value between 0 and 1
// TODO: only call this function when servo position changes
void setATSPosition(float val) {
    #if SIMULATE
        // Serial.println("(simulation) ATS position:" + String(val));
        return;
    #endif
    #if SKIP_ATS
        return;
    #endif
    val = 1.0f - val;
    float pos = (ats_max - ats_min) * val + ats_min;
    ATS_servo.write(int(pos));
    if (DEBUG) {Serial.println("ATS position set to " + String(pos));}
}

// Adjust the ATS based on the current altitude and desired apogee
void adjustATS() {
    if (millis() - launch_time > 18000) {
        // Retract ATS fully after 18 seconds
        setATSPosition(0.0);
        return;
    }

    if (altitude_filtered >= alt_target) {
        ats_position = 1.0;
    }
    else {
        // Calculate how wide to make the rocket so its hits its target altitude
        // 0 = 583.99^2 + 2 * a * 3927.15
        float target_area = (pow(velocity_filtered, 2)/(alt_target - altitude_filtered) - 2*GRAVITY)*ROCKET_MASS/(velocity_filtered*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT);
        Serial.println("Old ATS pos: " + String(target_area/ATS_MAX_SURFACE_AREA));
        // Adjust the ATS based on the target area
        //drag force
        // float Fd = 1/2 * ROCKET_DRAG_COEFFICIENT * (ROCKET_CROSS_SECTIONAL_AREA + MAX_FLAP_SURFACE_AREA*ats_position) * pow(velocity_filtered,2.0);
        // float inst_acceleration = Fd/ROCKET_MASS;
        float error = acceleration_filtered - TARGET_ACCELERATION; // positive if drag + gravity >= target, means if <, we are going TOO FAST and need to slow down
        // if error < 0, deploy, else, its fine
        if (error > 0){
          error = 0;
        } else {
          error = abs(error);
        }
        float adjustment = pid_factor(error, 0.03,0); // should normalize to 0 to 1

        // target_area -= ROCKET_CROSS_SECTIONAL_AREA;
        // ats_position = target_area/ATS_MAX_SURFACE_AREA;
        ats_position = adjustment;
    }

    // // This is last year's code to adjust the ATS; might need to be changed a bit
    // if (altitude_filtered > alt_target && ats_position < 1.0 && millis() - launch_time > 4500) {
    //     // If the rocket is above the target altitude, extend the ATS
    //     ats_position += 0.1;
    // } else if (altitude_filtered < alt_target && ats_position > 0.0 && millis() - launch_time > 4500) {
    //     // If the rocket is below the target altitude, retract the ATS
    //     ats_position -= 0.1;
    // }

    if (millis() - launch_time > 4500) {
        // Adjust ATS based on position
        setATSPosition(ats_position);
        Serial.println("ATS position: " + String(ats_position));
    }
}

// Test the ATS: fully extend and retract it, then detach the servo to save power
void testATS() {
    Serial.println("Testing ATS...");
    attachATS();
    setATSPosition(0.0f);
    delay(2000);
    setATSPosition(1.0f);  // Initial position
    delay(2000);
    setATSPosition(0.0f);  // Reset position
    delay(2000);
    detachATS();
    Serial.println("ATS Test Sucessful...");
}

// ***************** STATUS LED Methods *****************
void LEDLogging() {
    digitalWrite(LED_pin, HIGH);
}

void LEDSuccess() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_pin, LOW);
        delay(250);
        digitalWrite(LED_pin, HIGH);
        delay(250);
    }
}

void LEDError() {
    while (true) {
        digitalWrite(LED_pin, HIGH);
        delay(1000);
        digitalWrite(LED_pin, LOW);
        delay(1000);
    }
}

void LEDFlying() {
    digitalWrite(LED_pin, HIGH);
}

void LEDSetup(){
    pinMode(LED_pin, OUTPUT);
    digitalWrite(LED_pin, HIGH);
}

// If in simulation mode, update simulated sensor values from serial bus
#if SIMULATE

// Begin the simulation by waiting for the host program to start and give confirmation over serial
void startSimulation() {
    simulatedSensorValues->insert("altitude_raw", "0.0");
    simulatedSensorValues->insert("acceleration_raw", "0.0");
    simulatedSensorValues->insert("temperature", "0.0");

    // Await connection from host computer
    while (!Serial.readString()) {
        delay(10);
    }

    Serial.println("(simulation) Arduino is ready!");
}

// Reads the serial buffer for the most recent data sent by the hosts, and updates the simulatedSensorValues hashmap accordingly
void collectSimulatedData() {
    String serial_buffer = Serial.readString();
    String current_key, current_value = "";
    for (int i = 0; i < serial_buffer.length(); i++) {
        if (serial_buffer[i] == ':') {
            current_key = current_value;
            current_value = "";
        } else if (serial_buffer[i] == ',') {
            simulatedSensorValues->insert(current_key, current_value);
            Serial.println("Received "+current_key+", "+current_value);
            current_key = "";
            current_value = "";
        } else {
            current_value += serial_buffer[i];
        }
    }
}

float dataAsFloat(String measurement_name) {
    return simulatedSensorValues->search(measurement_name).toFloat();
}

// Updates the global measurement variables from the simulatedSensorValues hashmap, as opposed to trying to poll sensors
String getSimulatedMeasurements() {
        collectSimulatedData();
        
        float altitude_raw = dataAsFloat("altitude_raw");
        float acceleration_raw = dataAsFloat("acceleration_raw");

        filterData(altitude_raw, acceleration_raw);

        String movement_data = String(altitude_raw) + "," + String(acceleration_raw) + "," + String(altitude_filtered) + "," + String(velocity_filtered) + "," + String(acceleration_filtered);

        String time_data = String(millis() - start_time);

        String sensor_data = String(dataAsFloat("temperature")); // IK its a little redundant but it's for consistency

        return time_data + "," + movement_data + "," + sensor_data + "," + ats_position;
}
#endif
