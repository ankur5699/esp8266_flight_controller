#include "MPU9250.h"

MPU9250 mpu;

float yaw,pitch,roll;
    // float getAccX() const { return a[0]; }
    // float getAccY() const { return a[1]; }
    // float getAccZ() const { return a[2]; }
    // float getGyroX() const { return g[0]; }
    // float getGyroY() const { return g[1]; }
    // float getGyroZ() const { return g[2]; }
    // float getMagX() const { return m[0]; }
    // float getMagY() const { return m[1]; }
    // float getMagZ() const { return m[2]; }


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
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            yaw = mpu.getYaw();
            pitch = mpu.getPitch();
            roll = mpu.getRoll();
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() {
    Serial.print("Yaw:"); Serial.println(yaw);
    Serial.print("Pitch:"); Serial.println(pitch);
    Serial.print("Roll:"); Serial.println(roll);
}
