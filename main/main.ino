#include "MPU9250.h"

MPU9250 mpu;

float acc[3];
float gyro[3];
float mag[3];
float roll, pitch, yaw;
float dt = 0.025;

// Complementary filter variables
float alpha = 0.98; // Adjust this value based on your requirements

void convertAcc() {
  roll = atan2(acc[1], acc[2]) * (180 / PI);
  pitch = atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * (180 / PI);
}

void convertGyro() {
   // Adjust this value based on your loop timing
  roll = alpha * (roll + gyro[0] * dt) + (1 - alpha) * atan2(acc[1], acc[2]) * (180 / PI);
  pitch = alpha * (pitch + gyro[1] * dt) + (1 - alpha) * atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * (180 / PI);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
}

void loop() {
  if (mpu.update()) {
    // Read accelerometer, gyroscope, and magnetometer data
    acc[0] = mpu.getAccX();
    acc[1] = mpu.getAccY();
    acc[2] = mpu.getAccZ();
    gyro[0] = mpu.getGyroX();
    gyro[1] = mpu.getGyroY();
    gyro[2] = mpu.getGyroZ();
    mag[0] = mpu.getMagX();
    mag[1] = mpu.getMagY();
    mag[2] = mpu.getMagZ();

    // Apply complementary filter for sensor fusion
    convertGyro();

    // Combine magnetometer data
    float magHeading = atan2(mag[1], mag[0]) * (180 / PI);
    yaw = alpha * (yaw + gyro[2] * dt) + (1 - alpha) * magHeading;

    // Print the angles
    print_roll_pitch_yaw();
  }
}

void print_roll_pitch_yaw() {
    // Print the angles in a format for the serial plotter
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(yaw);
}
