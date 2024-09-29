/*
Carnegie Mellon Rocketry Club: Precision INstrumented Experimental Aerial Propulsion Payload for Low-altitude Exploration ("PINEAPPLE") data logger             (see what I did there lol)

This sketch is designed to read data from sensors described below, smooth and filter it, and write it to an SD card. The data is written in a CSV format, with each line containing both raw and filtered measurements.
This will also use IMU/accelerometer data to determine when the rocket has launched, and will begin logging data at that point. Additionally, when the rocket has landed, it will stop logging data and transmit the relevant measurements over radio.
Finally, this code is also responsible for monitoring the rocket's altitude and deploying the ATS as necessary to reach the desired apogee.

Sensor/device (model, pin #, protocol):
 - IMU/Accelerometer ()
 - Altimeter ()
 - Thermometer ()
 - Radio transmitter ()
 - STEMnaut interface ()
 - Battery voltage monitor ()
 - ATS Servo (???, 6, Arduino Servo library)

More project details tracked at: https://docs.google.com/document/d/17LliiDlGIH2ky337JQ54YeVqc5DDVWyw8OYpTEvQ4oI/edit

Made by the 2025 Avionics team :D
*/


// IMPORTANT CONSTANTS

// ⚠⚠⚠ IMPORTANT: SIMULATE = true will NOT actually gather data, only simulate it for testing purposes ⚠⚠⚠
// Don't forget to set to false before launch!!!!!
const bool SIMULATE = false;

// Whether to print debugging messages to the serial monitor (even if SIMULATE is off)
const bool DEBUG = true; 

// How frequently data should be collected (in milliseconds)
const int loop_target = 10; // 100 Hz

// Target altitude in feet
const float alt_target = 5000.0f; 



// LIBRARIES
#include <Servo.h>
#include <SD.h>

// 2024 sensor libraries
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_LSM6DSOX.h>


// PIN DEFINITIONS
const int ats_pin = 6;
const int LED_pin = 13;


// ATS SERVO PARAMETERS
Servo ATS_servo;
float ats_position = 0.0f;
const int ats_min = 0;
const int ats_max = 78;


// RADIO PARAMETERS



// SD CARD PARAMETERS
const int chip_select = 10;
bool sd_active = false;


// SENSOR OBJECTS AND PARAMETERS

// 2024 sensors
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP3XX bmp;


// MEASUREMENT CONSTANTS AND VARIABLES

// Keeps track of data until it is written to the SD card
String buffer;

// Number of measurements to take before writing to SD card
const int buffer_size = 500;

// Keeps track of time to make sure we are taking measurements at a consistent rate
unsigned long start_time, curr_time, timer, loop_time, prev_loop_time = 0;

// Filtered measurements shall be kept as global variables; raw data will be kept local to save memory
long altitude_filtered, velocity_filtered, acceleration_filtered;

// Remembers if the rocket has launched and landed
bool launched, landed;
unsigned long launch_time;
unsigned long land_time = 0;

// Acceleration threshold for launch detection
const float accel_threshold = 10.0f;
// Velocity threshold for landing detection
const float velocity_threshold = 0.1f;



//Sensor Initlization; SD Card Setup; Config. ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// We have two of these functions now hooray (?)
// bool initializeSDCard() {
//   Serial.print("Initializing SD card...");
//   if (!SD.begin(chip_select)) {
//     // TODO: maybe add some logic to retry initialization if it fails the first time
//     Serial.println("Card failed, or not present");
//     return false;
//   }
//   Serial.println("Card initialized.");
//   return true;
// }


// Sensors from 2024
bool setupBMP3XX() {
  if (!bmp.begin_I2C()) {
    Serial.print("BMP sensor is bad");
    return false;
  }
  return true;
}

bool setupBNO055() {
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  }
  return true;
}

// Test the ATS: fully extend and retract it, then detach the servo to save power
void configureServo() {
    attachATS();
    setATSPosition(1.0f);  // Initial position
    delay(2000);
    setATSPosition(0.0f);  // Reset position
    delay(2000);
    detachATS();
}


// Entry point to the program
// Initializes all sensors and devices, and briefly tests the ATS
void setup() {
    Serial.begin(115200);

    if (!initializeSDCard()) {
        // Consider retry logic or a safe shutdown
        // TODO: We need some visual indicator that the SD card failed to initialize (perhaps via the nice function that was already written)
        return;
    }

    Wire.begin();

    // Initialize sensors
    // These are the 2024 sensors; we need to update this for 2025
    if (!setupBMP3XX() || !setupBNO055()) {
        // Handle sensor initialization failure
        while (1);
    }

    // Test the ATS
    configureServo();

    pinMode(LED_pin, OUTPUT);
    start_time = millis();
    writeData("***********************************************************************************");
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
                if (DEBUG) {Serial.println("Rocket has landed!");}
                ATS_servo.detach();
            }
        }
    }
    if (launched) {
        // Write batch of data to SD card
        writeData(buffer);
    }

    if (landed) {
        // Transmit data over radio
        transmitData();
        // End the program
        return;
    }
}


// Function to handle loop timing: delays the loop to ensure it runs at a consistent rate
void run_timer() {
    long temp_time = millis() - prev_loop_time;
    if (temp_time < loop_target) {
        delayMicroseconds((loop_target - temp_time) * 1000);
    }
    curr_time = millis();
    timer = curr_time - start_time;
    loop_time = curr_time - prev_loop_time;
    prev_loop_time = curr_time;
}


// Initialize the SD card (returns 0 if successful, -1 if failed)
// Right now, tries to connect 10 times before going into an error state; could change so it keeps trying indefinitely
bool initializeSDCard() {
    if (!SIMULATE) {

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
                return -1;
            }
        }

        if (DEBUG) {Serial.println("SD card initialized successfully!");}

    }
    else {
        Serial.println("(simulation) Initializing SD card... SD card initialized successfully!");
    }

    sd_active = true;
    // Write CSV header to the file
    writeData("time,altitude_raw,acceleration_raw,altitude_filtered,velocity_filtered,acceleration_filtered,temperature,ats_position\n");
    return 0;
}


// Log data to the SD card in the file "datalog.txt"
void writeData(String text) {
    if (sd_active) {
        File data_file = SD.open("datalog.txt", FILE_WRITE);
        if (data_file) {
            data_file.println(text);
            data_file.close();
        } else {
            // Could leave this out so the program tries to keep logging data even if it fails once
            // sd_active = false;

            Serial.println("Error opening datalog.txt");
        }
    } else {
        // If the SD card has not connected successfully
        if (DEBUG) {Serial.println("SD logging failed. Continuing without logging.");}
    }
}


// Get measurements from sensors and return them as a CSV string, also updating global variables as necessary
// Format: time, altitude_raw, acceleration_raw, altitude_filtered, velocity_filtered, acceleration_filtered, temperature, ats_position
String getMeasurements() {
    float altitude_raw, acceleration_raw;

    altitude_raw = readAltimeter();
    acceleration_raw = readIMU();

    filterData(altitude_raw, acceleration_raw);

    String movement_data = String(altitude_raw) + "," + String(acceleration_raw) + "," + String(altitude_filtered) + "," + String(velocity_filtered) + "," + String(acceleration_filtered);

    String time_data = String(millis() - start_time);

    String sensor_data = String(readThermometer());

    return time_data + "," + movement_data + "," + sensor_data + "," + ats_position;
}


// Filter raw data and store it in global variables; still need to implement
void filterData(float alt, float acc) {
    altitude_filtered = alt;
    velocity_filtered = 0;
    acceleration_filtered = acc;
}


// Read altitude from altimeter and return it
float readAltimeter() {
    float altimeter_data = 0.0f;
    if (DEBUG) {Serial.println("Altimeter: " + String(altimeter_data));}
    return altimeter_data;
}


// Read IMU data, get vertical acceleration, and return it
float readIMU() {
    float imu_data = 0.0f;
    if (DEBUG) {Serial.println("IMU: " + String(imu_data));}
    return imu_data;
}


// Read temperature from thermometer and return it
float readThermometer() {
    float thermometer_data = 0.0f;
    if (DEBUG) {Serial.println("Thermometer: " + String(thermometer_data));}
    return thermometer_data;
}


// Detect if the rocket has landed: if the rocket has been reasonably still for 5 seconds, it is considered landed
bool detectLanding() {
    if (velocity_filtered < velocity_threshold) {
        land_time = millis();
    }
    else {
        land_time = 0;
    }

    if (land_time != 0 && millis() - land_time > 5000) {
        // TODO: might want to add some more conditions here; if this messes up, we transmit data early and lose the competition :(
        return true;
    }

    return false;
}


// Turns on the ATS servo
void attachATS() {
    ATS_servo.attach(ats_pin);
}

// Turns off the ATS servo (to save power)
void detachATS() {
    ATS_servo.detach();
}

// Sets how far the ATS is extended based on a value between 0 and 1
void setATSPosition(float val) {
    float pos = (ats_max - ats_min) * val + ats_min;
    ATS_servo.write(int(pos));
    if (DEBUG) {Serial.println("ATS position set to " + String(pos));}
}

// Adjust the ATS based on the current altitude and desired apogee
void adjustATS() {
    // This is last year's code to adjust the ATS; might need to be changed a bit
    if (altitude_filtered > alt_target && ats_position < 1.0 && millis() - launch_time > 4500) {
        // If the rocket is above the target altitude, extend the ATS
        ats_position += 0.1;
    } else if (altitude_filtered < alt_target && ats_position > 0.0 && millis() - launch_time > 4500) {
        // If the rocket is below the target altitude, retract the ATS
        ats_position -= 0.1;
    }

    if (millis() - launch_time > 18000) {
        // Retract ATS fully after 18 seconds
        setATSPosition(0.0);
    } else if (millis() - launch_time > 4500) {
        // Adjust ATS based on position
        setATSPosition(ats_position);
    }
}


// Functions for status LED
void LEDLogging() {
    digitalWrite(LED_pin, HIGH);
}

void LEDSuccess() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_pin, HIGH);
        delay(250);
        digitalWrite(LED_pin, LOW);
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


// Transmit data over radio
void transmitData() {
    if (DEBUG) {Serial.println("Transmitting data over radio...");}
}
