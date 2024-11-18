/*
Carnegie Schmellon Rocketry Club: Precision INstrumented Experimental Aerial Propulsion Payload for Low-altitude Exploration ("PINEAPPLE") data logger             (see what I did there lol)

This sketch is designed to read data via SPI from sensors described below, smooth and filter it, and predict the rocket's apogee based on the data, adjusting the ATS system to get as close as possible to the desired altitude.
This will also use IMU/accelerometer data to determine when the rocket has launched, and will begin logging data to an onboard SD card. The data is written in a CSV format, with each line containing both raw and filtered measurements. Additionally, when the rocket has landed, it will stop logging data.

Sensor/device (model, pin #, protocol):
 - IMU/Accelerometer (LSM6DSOX, 9, SPI)
 - Altimeter (BMP390, 8, SPI)
 - Thermometer (LSM6DSOX (built in to IMU), 9, SPI)
 - ATS Servo (???, 6, Arduino Servo library)

SPI pins are the default hardware SPI pins on the Teensy 4.1 (MISO = 12, MOSI = 11, SCK = 13)

DEBUGGING: if the onboard LED flashes rapidly a few times, then stays on, everything has been initialized correctly. If the LED blinks slowly, there is an error with the SD card or one of the sensors.

More project details tracked at: https://docs.google.com/document/d/17LliiDlGIH2ky337JQ54YeVqc5DDVWyw8OYpTEvQ4oI/edit

Made by the 2025 Avionics team :D
*/

// TODO: Kalman filter

// IMPORTANT CONSTANTS (Using IPS)

// ⚠⚠⚠ IMPORTANT: SIMULATE = true will NOT actually gather data, only simulate it for testing purposes ⚠⚠⚠
// Don't forget to set to false before launch!!!!!
#define SIMULATE false

// Whether we are flying the subscale rocket or not (no ATS servo)
#define SUBSCALE true

// Whether to print debugging messages to the serial monitor (even if SIMULATE is off)
const bool DEBUG = true;

// How frequently data should be collected (in milliseconds)
const int loop_target = 25;

// Target altitude in feet
#if SUBSCALE
    const float alt_target = 2000.0f;
#else
    const float alt_target = 5000.0f;
#endif


// LIBRARIES
#include <Servo.h>
#include <SD.h>


// Simulation mode libraries
#if SIMULATE
    #include <Dictionary.h>
    Dictionary *simulatedSensorValues = new Dictionary();
#endif

// sensor libraries
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>


// PIN DEFINITIONS
const int ats_pin = 6;
const int LED_pin = LED_BUILTIN;
// const int IMU_chip_select = 9;    // SOX
const int altimeter_chip_select = 1;     // BMP
// We shouldn't need to define these if the Teensy has dedicated hardware SPI pins
// const int MIS0 = 12;
// const int MOSI = 11;
// const int SCK = 13;


// ATS SERVO PARAMETERS
Servo ATS_servo;
float ats_position = 0.0f;
const int ats_min = 0;
const int ats_max = 78;


// SD CARD PARAMETERS
const int chip_select = 0;
bool sd_active = false;
String file_name = "subscl_2.txt"; // ⚠⚠⚠ FILE NAME MUST BE 8 CHARACTERS OR LESS OR ARDUINO CANNOT WRITE IT (WHY?!?!) ⚠⚠⚠


// SENSOR OBJECTS AND PARAMETERS

Adafruit_BMP3XX bmp;     // Altimeter
Adafruit_LSM6DSOX sox;    // IMU
sensors_event_t accel, gyro, temp;


// MEASUREMENT CONSTANTS AND VARIABLES

// Keeps track of data until it is written to the SD card
String buffer;

// Number of measurements to take before writing to SD card
const int buffer_size = 50;

// Keeps track of time to make sure we are taking measurements at a consistent rate
unsigned long start_time, curr_time, timer, loop_time, prev_loop_time = 0;

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
float gravity_normal_vector[3] = {0.0, 0.0, 0.0};


// Acceleration threshold for launch detection
const float accel_threshold = 20.0f;
// Velocity threshold for landing detection
const float velocity_threshold = 0.1f;


// UNITS
#define SEALEVELPRESSURE_HPA (1013.25)
#define METERS_TO_FEET 3.28084

#define ATMOSPHERE_FLUID_DENSITY 0.076474f // lbs/ft^3
#define ROCKET_DRAG_COEFFICIENT 0.75f   // TODO: figure out actual value
#define ROCKET_CROSS_SECTIONAL_AREA 0.1f // ft^2
#define ROCKET_MASS 3.0f // lbs
#define ATS_MAX_SURFACE_AREA 1.5*ROCKET_CROSS_SECTIONAL_AREA // TODO: figure out actual value
#define g 32.174f // ft/s^2

// Entry point to the program
// Initializes all sensors and devices, and briefly tests the ATS
void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");


    #if SIMULATE
        startSimulation();
    #endif

    // Initialize SD card
    if (!initializeSDCard()) {
        // Tries to initialize the SD card 10 times before giving up
        // If this fails, something is wrong: stops execution and blinks the LED to indicate an error
        LEDError();
    }

    // Initialize sensors
    if (!setupSensors()) {
        // If one of the sensors doesn't connect, stop execution and blinks onboard LED to indicate an error
        Serial.println("Sensor setup failed. Aborting.");
        LEDError();
    }

    // Test the ATS
    testATS();

    pinMode(LED_pin, OUTPUT);
    start_time = millis();
    Serial.println("Arduino is ready!");
}


// Repeats indefinitely after setup() is finished
// Will measure data, filter it, and write it to the SD card from when the rocket launches until it lands
void loop() {
    buffer = "";

    for (int i = 0; i < buffer_size; i++) {

        // Run timer to ensure loop runs at a consistent rate
        run_timer();

        // Get measurements from sensors and add them to the buffer
        buffer += getMeasurements();
        buffer += "\n";

        // Detect launch based on acceleration threshold
        if (acceleration_filtered > accel_threshold && !launched) {
            // Write CSV header to the file
            writeData("***************** START OF DATA ***************** TIME SINCE BOOT: " + String(millis()) + " ***************** TICK SPEED: " + String(loop_target) + "ms\n");
            writeData("time, pressure (hPa), altitude_raw (ft), acceleration_raw_x (ft/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, altitude_filtered (ft), velocity_filtered (ft/s), acceleration_filtered (ft/s^2), temperature (from IMU, degrees C), ats_position (servo degrees)\n");

            launched = true;
            launch_time = millis();
            if (DEBUG) {Serial.println("Rocket has launched!");}

            // Bring the ATS back online
            attachATS();
            setATSPosition(0.0);

            // Set status LED
            LEDLogging();
        }

        // If the rocket has launched, adjust the ATS as necessary, and detect whether the rocket has landed
        if (launched) {
            adjustATS();

            if (detectLanding()) {
                landed = true;
                ATS_servo.detach();
            }
        }
    }
    if (launched) {
        // Write batch of data to SD card
        writeData(buffer);
    }
    else {
        // If rocket has not launched yet, assume it's still on the pad, and find which way is down
        updatePrelaunchNormalVector();
    }

    if (landed) {
        // End the program
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
    curr_time = millis();
    timer = curr_time - start_time;
    loop_time = curr_time - prev_loop_time;
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


// Sensor setup functions

// Initialize all sensors and return true if successful, false if any have failed
bool setupSensors() {
  #if SIMULATE
    Serial.println("(simulation) Sensors connected successfully!");
    return true;
  #endif

  if (setupBMP3XX() && setupLSM6DSOX()) {
    if (DEBUG) {Serial.println("Sensors initialized successfully!");}
    return true;
  }
  return false;
}

// Altimeter
bool setupBMP3XX() {
  if (!bmp.begin_SPI(altimeter_chip_select)) {
    Serial.println("Unable to connect to altimeter");
    return false;
  }
  return true;
}

// IMU
bool setupLSM6DSOX() {
  if (!sox.begin_I2C()) {
    Serial.println("Unable to connect to IMU");
    return false;
  }
  return true;
}



// Get measurements from sensors and return them as a CSV string, also updating global variables as necessary
// Format: time, pressure (hPa), altitude_raw (m), acceleration_raw_x (m/s^2), acceleration_raw_y, acceleration_raw_z, gyro_x (radians/s), gyro_y, gyro_z, altitude_filtered, velocity_filtered, acceleration_filtered, temperature (from IMU, degrees C), ats_position (degrees)
String getMeasurements() {
    #if SIMULATE
        // If simulation mode is ON, get simulated data from serial bus instead
        return getSimulatedMeasurements();
    #endif

    float altitude_raw, acceleration_raw;

    updateAltimeter();
    updateIMU();
    altitude_raw = readAltimeter();
    acceleration_raw = readIMU();

    filterData(altitude_raw, acceleration_raw);

    String movement_data = String(bmp.pressure/100.0) + "," + String(bmp.readAltitude(SEALEVELPRESSURE_HPA)) + "," + String(accel.acceleration.x*METERS_TO_FEET) + "," + String(accel.acceleration.y*METERS_TO_FEET) + "," + String(accel.acceleration.z*METERS_TO_FEET) + "," + String(gyro.gyro.x) + "," + String(gyro.gyro.y) + "," + String(gyro.gyro.z) + "," + String(altitude_filtered) + "," + String(velocity_filtered) + "," + String(acceleration_filtered);

    String time_data = String(millis() - start_time);

    String sensor_data = String(readThermometer());
    // Serial.println(time_data + "," + movement_data + "," + sensor_data + "," + String(ats_position));
    return time_data + "," + movement_data + "," + sensor_data + "," + String(ats_position);
}


// Filter raw data and store it in global variables; still need to implement
void filterData(float alt, float acc) {

    altitude_filtered = alt;
    velocity_filtered = (altitude_filtered - previous_velocity_filtered) / (loop_time/1000);
    acceleration_filtered = acc;

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


// Read altitude from altimeter (in meters) and return it
float readAltimeter() {
    float altimeter_data = bmp.readAltitude(SEALEVELPRESSURE_HPA) * METERS_TO_FEET;
    // float altimeter_data = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    // if (DEBUG) {Serial.println("Altimeter: " + String(altimeter_data));}
    return altimeter_data;
}

void updateAltimeter() {
    bmp.performReading();
}

// Poll all measurements from IMU at once
void updateIMU() {
    sox.getEvent(&accel, &gyro, &temp);
}

// Read IMU data, get vertical acceleration (in meters/second^2), and return it
float readIMU() {
    // Assume the IMU is positioned at a strange angle, but that all relevant acceleration is upward
    float imu_data = pow(pow(accel.acceleration.x,2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2), 0.5);
    // If the net acceleration is in the same direction as gravity, make it negative for "downward"
    if (accel.acceleration.x * gravity_normal_vector[0] + accel.acceleration.y * gravity_normal_vector[1] + accel.acceleration.z * gravity_normal_vector[2] >= 0) {
        imu_data = -imu_data;
    }
    // Convert to feet
    imu_data = imu_data * METERS_TO_FEET;
    // if (DEBUG) {Serial.println("IMU: " + String(imu_data));}
    return imu_data;
}

void updatePrelaunchNormalVector() {
    // Get the average acceleration vector (from gravity) before launch
    gravity_normal_vector[0] = accel.acceleration.x*METERS_TO_FEET;
    gravity_normal_vector[1] = accel.acceleration.y*METERS_TO_FEET;
    gravity_normal_vector[2] = accel.acceleration.z*METERS_TO_FEET;
}

// Read temperature from thermometer (in degrees C) and return it
float readThermometer() {
    float thermometer_data = temp.temperature;
    // if (DEBUG) {Serial.println("Thermometer: " + String(thermometer_data));}
    return thermometer_data;
}


// Detect if the rocket has landed: if the rocket has been reasonably still for 5 seconds, it is considered landed
bool detectLanding() {
    if (launch_time != 0 && millis() - launch_time > 60000) {
        return true;
    }


    if (velocity_filtered < velocity_threshold) {
        land_time = millis();
    }
    else {
        land_time = 0;
    }

    if (land_time != 0 && millis() - land_time > 5000) {
        return true;
    }

    return false;
}


// ATS METHODS

// Turns on the ATS servo
void attachATS() {
    #if SIMULATE
        return;
    #endif
    #if SUBSCALE
        return;
    #endif

    ATS_servo.attach(ats_pin);
}

// Turns off the ATS servo (to save power)
void detachATS() {
    #if SIMULATE
        return;
    #endif
    #if SUBSCALE
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
    #if SUBSCALE
        return;
    #endif

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
        float target_area = (pow(velocity_filtered, 2)/(alt_target - altitude_filtered) - 2*g)*ROCKET_MASS/(velocity_filtered*ATMOSPHERE_FLUID_DENSITY*ROCKET_DRAG_COEFFICIENT);
        // Adjust the ATS based on the target area
        target_area -= ROCKET_CROSS_SECTIONAL_AREA;
        ats_position = target_area/ATS_MAX_SURFACE_AREA;
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
    }
}

// Test the ATS: fully extend and retract it, then detach the servo to save power
void testATS() {
    attachATS();
    setATSPosition(1.0f);  // Initial position
    delay(2000);
    setATSPosition(0.0f);  // Reset position
    delay(2000);
    detachATS();
}



// STATUS LED METHODS
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


// If in simulation mode, updated simulated sensor values from serial bus
#if SIMULATE
void startSimulation() {
    simulatedSensorValues->insert("altitude_raw", "0.0");
    simulatedSensorValues->insert("acceleration_raw", "0.0");
    simulatedSensorValues->insert("temperature", "0.0");
    Serial.println("(simulation) Arduino is ready!");
}

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
