/*
  SD Card Data Logger for ATS Kalman Filter

  This project logs data from an altimeter and IMU (Inertial Measurement Unit), processes it using
  a Kalman filter for altitude estimation, and controls an ATS (Altitude Thrust System) actuator based
  on predicted altitude. The data is logged to an SD card using a Teensy 4.1.

  Sensors:
  * BMP3XX barometric sensor for altitude
  * BNO055 IMU for gyroscope, acceleration, magnetometer, and gravity measurements

  ATS Control:
  * Servo controlling actuator position for altitude adjustment

  Circuit:
  * Analog sensors connected to analog pins A0, A1, A2
  * SD card connected via SPI to Teensy 4.1
  * ATS actuator connected to digital pin 6

  Written by Jeffery John for ATS Kalman Filter
  Modified 1 Feb 2024
*/

#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// Constants
const int chip_select = BUILTIN_SDCARD;      // SD card chip select pin
const int led_pin = 13;                      // Onboard LED pin
const int ats_pin = 6;                       // Servo pin for ATS
const int ats_min = 0;                       // ATS minimum position (fully retracted)
const int ats_max = 78;                      // ATS maximum position (fully extended)
const int alt_target = 5000;                 // Target altitude in feet
const int buf_size = 500;                    // Buffer size for data logging

const float meter_to_foot = 3.2808399;       // Conversion factor: meters to feet
const float accel_threshold = 10.0;          // Acceleration threshold for detecting launch

// Kalman Filter Variables
float process_noise = 0.025;                 // Process noise covariance
float measurement_noise = 0.5;               // Measurement noise covariance
float altitude_estimate = 0.0;               // Current altitude estimate
float altitude_error_estimate = 3.0;         // Error estimate for altitude

// Sensor Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);    // IMU sensor
Adafruit_BMP3XX bmp;                                // Barometric sensor

// Timing and Buffer Variables
long start_time, curr_time, prev_loop_time, loop_target = 10;
float loop_time = 0;
String buf = "";                                 // Data buffer

// Servo Object
Servo ATS;

// State Variables
bool sd_active = true;
bool launched = false;
bool attached = false;
unsigned long launch_time = 0;

// Kalman Filter Update Function
void update_kalman_filter(float measurement) {
    altitude_estimate += 0; // Assume constant acceleration
    altitude_error_estimate += process_noise;

    // Update step
    float kalman_gain = altitude_error_estimate / (altitude_error_estimate + measurement_noise);
    altitude_estimate += kalman_gain * (measurement - altitude_estimate);
    altitude_error_estimate = (1 - kalman_gain) * altitude_error_estimate + process_noise;
}

// Get the current altitude estimate
float get_altitude_estimate() {
    return altitude_estimate;
}

// Setup function - runs once when the program starts
void setup() {
    Serial.begin(115200);                         // Start serial communication
    Serial.print("Initializing SD card...");

    // Initialize the SD card
    if (!SD.begin(chip_select)) {
        Serial.println("Card failed, or not present");
        setup();
    }
    Serial.println("Card initialized.");

    Wire.begin();                                  // Start I2C communication

    if (!bmp.begin_I2C()) {                        // Initialize BMP sensor
        Serial.println("BMP sensor initialization failed.");
        while (1);
    }

    // Initialize altitude reference
    altitude_estimate = bmp.readAltitude(1015.5) * meter_to_foot;

    if (!bno.begin()) {                            // Initialize BNO055 IMU sensor
        Serial.println("No BNO055 detected. Check wiring or I2C address!");
        while (1);
    }

    ATS.attach(ats_pin);                           // Attach servo to ATS pin
    set_ats_position(1);                           // Initialize ATS position
    delay(2000);
    set_ats_position(0);
    delay(2000);
    ATS.detach();                                  // Detach ATS after initialization

    pinMode(led_pin, OUTPUT);                      // Set LED pin as output
    start_time = millis();                         // Start timer
    prev_loop_time = start_time;
    write_data("***********************************************************************************");
}

// Loop function - runs continuously
void loop() {
    buf = "";                                      // Clear data buffer

    for (int i = 0; i < buf_size; i++) {
        run_timer();                               // Manage loop timing

        read_altimeter();                          // Read altitude sensor
        read_imu();                                // Read IMU sensor

        filter_altimeter();                        // Apply filter to altimeter data
        filter_accel(&linear_accel_data);          // Apply filter to accelerometer data
        filter_velocity();                         // Apply filter to velocity data
        velocity_fusion();                         // Fuse velocity data
        make_prediction();                         // Make altitude prediction

        // Construct data string for logging
        String s1 = String(timer) + ", " + String(loop_time) + ", " + String(altitude) + ", " + String(altimeter_filtered) + ", " + String(velocity) + ", " + String(velocity_filtered);
        String s2 = String(prediction) + ", " + String(prediction_filtered) + ", " + String(acceleration) + ", " + String(acceleration_filtered);
        String s3 = print_event(&linear_accel_data);
        String s4 = print_event(&accelerometer_data);
        String s5 = print_event(&gravity_data);
        String s6 = print_event(&ang_velocity_data);
        String s7 = print_event(&magnetometer_data);

        String data = s1 + ", " + s2 + ", " + s3 + ", " + s4 + ", " + s5 + ", " + s6 + ", " + s7 + ", " + String(ats_position);
        if (i != buf_size - 1) {
            data += "\n";
        }

        buf += data;

        // Detect launch based on acceleration threshold
        if (acceleration_filtered > accel_threshold && !launched) {
            launched = true;
            digitalWrite(led_pin, HIGH);
            launch_time = millis();
        }

        // Attach ATS after launch and manage its position
        if (!attached && millis() - launch_time > 4500) {
            ATS.attach(ats_pin);
            set_ats_position(0.0);
            attached = true;
        }

        if (launched) {
            if (prediction_filtered > alt_target && ats_position < 1.0 && millis() - launch_time > 4500) {
                ats_position += 0.1;
            } else if (prediction_filtered < alt_target && ats_position > 0.0 && millis() - launch_time > 4500) {
                ats_position -= 0.1;
            }

            if (millis() - launch_time > 18000) {
                // Retract ATS fully after 18 seconds
                set_ats_position(0.0);
            } else if (millis() - launch_time > 4500) {
                // Adjust ATS based on position
                set_ats_position(ats_position);
            }
        }
    }

    if (launched) {
        // Write buffered data to SD card
        write_data(buf);
    }
}

// Function to handle loop timing
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

// Function to log data to SD card
void write_data(String text) {
    if (sd_active) {
        File data_file = SD.open("datalog.txt", FILE_WRITE);
        if (data_file) {
            data_file.println(text);
            data_file.close();
        } else {
            sd_active = false;
            Serial.println("Error opening datalog.txt");
        }
    } else {
        Serial.println("SD logging failed. Continuing without logging.");
    }
}

// Function to read data from the IMU sensor
void read_imu() {
    bno.getEvent(&ang_velocity_data, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linear_accel_data, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometer_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometer_data, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravity_data, Adafruit_BNO055::VECTOR_GRAVITY);
}

// Function to print sensor event data
String print_event(sensors_event_t *event) {
    // Initialize with dummy values for debugging
    double x = -1000000, y = -1000000, z = -1000000;

    // Switch based on event type
    switch (event->type) {
        case SENSOR_TYPE_ACCELEROMETER:
            x = event->acceleration.x;
            y = event->acceleration.y;
            z = event->acceleration.z;
            break;

        case SENSOR_TYPE_MAGNETIC_FIELD:
            x = event->magnetic.x;
            y = event->magnetic.y;
            z = event->magnetic.z;
            break;

        case SENSOR_TYPE_ORIENTATION:
            x = event->orientation.x;
            y = event->orientation.y;
            z = event->orientation.z;
            break;

        case SENSOR_TYPE_GYROSCOPE:
            x = event->gyro.x;
            y = event->gyro.y;
            z = event->gyro.z;
            break;

        case SENSOR_TYPE_GRAVITY:
            x = event->acceleration.x;
            y = event->acceleration.y;
            z = event->acceleration.z;
            break;

        default:
            // Keep the initialized dummy values if event type is unrecognized
            break;
    }

    return String(x) + ", " + String(y) + ", " + String(z);
}

// Function to read data from the altimeter
void read_altimeter() {
    altitude = bmp.readAltitude(1015.5) * meter_to_foot;
}