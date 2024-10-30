// imports
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <SPI.h>

#define BMP_CS 1
#define SEALEVELPRESSURE_HPA (1013.25)

// sensor objs
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX lsm6ds;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // pinMode(3, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  // BMP390 setup
  while (!bmp.begin_SPI(1)) {
    Serial.println("no altimeter connection");
  }

  // IMU setup
  while (!lsm6ds.begin_I2C()) {
    Serial.println("no IMU connection");
  }

  Serial.println("initialization done.");
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:

  sensors_event_t accel, gyro, temp;

  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel, &gyro, &temp);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x, 4);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z, 4);
  Serial.println(" \tm/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro  X: ");
  Serial.print(gyro.gyro.x, 4);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z, 4);
  Serial.println(" \tradians/s ");

  // Temp
  Serial.print("\t\tTemp   :\t\t\t\t\t");
  Serial.print(temp.temperature);
  Serial.println(" \tdeg C");
  Serial.println();

  // Altimeter reading
    if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}
