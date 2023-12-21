#include "MPU9250.h"
#include <Servo.h>

#define MotorPin 15

Servo ESC;
MPU9250 mpu;
float value = 0;

float acc[3];
float gyro[3];
float mag[3];
float roll, pitch, yaw;
float dt = 0.005;

// Complementary filter variables
float alpha = 0.80; // Adjust this value based on your requirements


void init_and_calibrate(bool calibrate_off)
{
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    if (!calibrate_off){
    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    // Serial.println("Mag calibration will start in 5sec.");
    // Serial.println("Please Wave device in a figure eight until done.");
    // delay(5000);
    // mpu.calibrateMag();

    print_calibration();
    // mpu.verbose(false);
    }
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    // Serial.println("mag bias [mG]: ");
    // Serial.print(mpu.getMagBiasX());
    // Serial.print(", ");
    // Serial.print(mpu.getMagBiasY());
    // Serial.print(", ");
    // Serial.print(mpu.getMagBiasZ());
    // Serial.println();
    // Serial.println("mag scale []: ");
    // Serial.print(mpu.getMagScaleX());
    // Serial.print(", ");
    // Serial.print(mpu.getMagScaleY());
    // Serial.print(", ");
    // Serial.print(mpu.getMagScaleZ());
    // Serial.println();
}

void convertAcc() {
  roll = atan2(acc[1], acc[2]) * (180 / PI);
  pitch = atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * (180 / PI);
}

void convertGyro() {
   // Adjust this value based on your loop timing
  roll = alpha * (roll + gyro[0] * dt) + (1 - alpha) * atan2(acc[1], acc[2]) * (180 / PI);
  pitch = alpha * (pitch + gyro[1] * dt) + (1 - alpha) * atan2(-acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * (180 / PI);
}

float* manual_calibration(int N){
  float offset[2];
  for (int i=0; i<N; i++){
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
      convertGyro();
      offset[0] += pitch;
      offset[1] += roll;
    }

  }
  offset[0] /= N;
  offset[1] /= N;
  return offset;

}


void setup() {
  init_and_calibrate(false);
  ESC.attach(MotorPin,1000,2000);
}

void loop() {
  int i = 0;
  // float* offset_out = manual_calibration(100);
  // float offset_out[2] = {47.71, 48.21};
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

    // subtract_offset();
    // Apply complementary filter for sensor fusion
    convertGyro();

    // Combine magnetometer data
    float magHeading = atan2(mag[1], mag[0]) * (180 / PI);
    yaw = alpha * (yaw + gyro[2] * dt) + (1 - alpha) * magHeading;

    // pitch -= offset_out[0];
    // roll -= offset_out[1];


    // Print the angles
    print_roll_pitch_yaw();
  
}
    value = abs(pitch);
    if (value <= 1)
      value = 0;
    else if (value >= 45)
      value = 45;

    value = map(value, 0, 45, 0, 180);
    ESC.write(value);
    

}

void print_roll_pitch_yaw() {
    // Print the angles in a format for the serial plotter
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println(yaw);
}
