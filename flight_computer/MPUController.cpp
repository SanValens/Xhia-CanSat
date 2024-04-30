#include <Arduino.h>
#include "MPUController.h"
#include <Wire.h>

MPUController::MPUController() {

}

bool MPUController::begin() {
  Wire.begin(32, 33); //SDA, SCL respectivamente

  //Set power mode:
  Wire.beginTransmission(MPU9ADD);  
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  //Set General configuration
  Wire.beginTransmission(MPU9ADD);
  Wire.write(0x1A);
  Wire.write(0b00000101);
  Wire.endTransmission();

  //Set config for gyro
  Wire.beginTransmission(MPU9ADD);
  Wire.write(0x1B);
  Wire.write(0b00001000);
  Wire.endTransmission();

  //Set config for accel
  Wire.beginTransmission(MPU9ADD);
  Wire.write(0x1C);
  Wire.write(0b00010000);
  Wire.endTransmission();

  //Set config for accel #2
  Wire.beginTransmission(MPU9ADD);
  Wire.write(0x1D);
  Wire.write(0b00000101);
  Wire.endTransmission();;
  
  if(!verify_connection()){
    Serial.println("MPU Connection failed");
    return 0;
  }
  Serial.println("MPU Connection confirmed");
  return 1;
}

bool MPUController::verify_connection() {
  Wire.beginTransmission(MPU9ADD);
  Wire.write(0x75);  //Register holding WHO AM I data
  Wire.endTransmission();

  Wire.requestFrom(MPU9ADD, 1);    //Request a byte
  if (Wire.available() == 1) {  
    if(int(Wire.read()) == 113){
      return 1;
    } else {
      return 0;
    }
  }
}


int MPUController::verify_magnetometer() {

  for(int i = 12; i < 16; i++) {
    Wire.beginTransmission(i);
    Wire.write(0x00);  //Register holding WHO AM I data
    Wire.endTransmission();  
    Wire.requestFrom(i, 1);
    if(Wire.read() == 72) {
      return i;
    }
  }
  Serial.println("No address worked for the magnetometer. IDK WHAT CAN YOU DO");
  while(1);
}

void MPUController::mag_update() {
  Wire.beginTransmission(MAGADD);
  Wire.write(0x3);
  Wire.endTransmission();
  Wire.requestFrom(MAGADD, 7);
  if (Wire.available() == 7) {
    uint8_t raw_data[7];
    int16_t mag_raw[3];
    for (int i = 0; i < 7; i++) {
      raw_data[i] = Wire.read();
    }
    mag_raw[0] = raw_data[0] | ((int16_t)raw_data[1] << 8);
    mag_raw[1] = raw_data[2] | ((int16_t)raw_data[3] << 8);
    mag_raw[2] = raw_data[4] | ((int16_t)raw_data[5] << 8);
    mag_data[0] = (float)(mag_raw[0] * 0.15 * adjust_mag_coeff[0]);
    mag_data[1] = (float)(mag_raw[1] * 0.15 * adjust_mag_coeff[1]);
    mag_data[2] = (float)(mag_raw[2] * 0.15 * adjust_mag_coeff[2]);
  }
  
}

void MPUController::calibrate_mpu() {
  for(int i = 0; i < 3; i++) {
    a_offset[i] = 0.0;
    g_offset[i] = 0.0;
  }
  calibrate_accel();
  calibrate_gyro();
}

void MPUController::calibrate_accel() {
  float sum[] = {0.0,0.0,0.0};
  float counter = 0.0;
  Serial.println("Calibration started, leave the device on a flat surface");
  delay(100);
  for (int i = 0; i < 100; i++) {
    accel_update();
    counter += 1.0;
    for (int j = 0; j < 3; j++) {
      sum[j] = sum[j] + accel_data[j];
    }
    delay(10);
  }
  for (int j = 0; j < 3; j++) {
    a_offset[j] = sum[j] / counter;
  }
  a_offset[2] = -1.0 + a_offset[2];
  Serial.println("Acceleration calibration finished");
}

void MPUController::calibrate_gyro() {
  float sum[] = { 0, 0, 0 };
  float counter = 0;
  for (int i = 0; i < 500; i++) {
    counter += 1.0;
    gyro_update();
    for (int j = 0; j < 3; j++) {
      sum[j] = sum[j] + gyro_data[j];
    }
    delay(10);
  }
  for (int j = 0; j < 3; j++) {
    g_offset[j] = sum[j] / counter;
  }
  Serial.println("Calibration finished");
}

void MPUController::gyro_update() {
  Wire.beginTransmission(MPU9ADD);
  Wire.write(0x43);  //Register holding gyroscope, we indicate we are going to use this one
  Wire.endTransmission();

  Wire.requestFrom(MPU9ADD, 6);    //Request 6 bytes from Gyroscope measurments register. Meaning 48 bits
  if (Wire.available() == 6) {  //Make sure i now have 6 bytes in buffer
    for (int i = 0; i < 3; i++) {
      gyro_data[i] = (float(int16_t(Wire.read() << 8 | Wire.read())) / mpu_resolutions[1]) - g_offset[i];
    }
    //Ergo when i call .read() 6 times, i finish with no bytes in the buffer
  }
}

void MPUController::accel_update() {
  Wire.beginTransmission(MPU9ADD);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU9ADD, 6);
  if (Wire.available() == 6) {
    for (int i = 0; i < 3; i++) {
      accel_data[i] = ((float(int16_t(Wire.read() << 8 | Wire.read()))) / mpu_resolutions[0]) - a_offset[i];
    }
  }
}

void MPUController::angle_update_by_acc() {
  accel_update();
  angle_data_by_acc[0] = atan(accel_data[1]/sqrt((accel_data[0]*accel_data[0])+(accel_data[2]*accel_data[2]))) * 180/3.141592;
  angle_data_by_acc[1] = -atan(accel_data[0]/sqrt((accel_data[1]*accel_data[1])+(accel_data[2]*accel_data[2]))) * 180/3.141592;
}

void MPUController::angle_update_kalman() {
  if (last_millis_kalman == 0) {
    last_millis_kalman = millis();
  }
  float t = (millis() - last_millis_kalman);
  angle_update_by_acc();
  //Un poco mas de entendimiento con apartado Kalman Filter en una dimensión en mi cuaderno de programación
  //Ecuaciones de la 1 a la 5:
  for(int i = 0; i < 3; i++){
    angle_data[i] = angle_data[i] + (t / 1000) * gyro_data[i];  //Calculo por integración.
    if(i == 0 || i == 1) {
      incerteza_kalman[i] = incerteza_kalman[i] + ((t / 1000) * (t / 1000) * 4 * 4);
      float k_gain = incerteza_kalman[i] * 1 / (1 * incerteza_kalman[i] + 3 * 3);
      angle_data[i] = angle_data[i] + k_gain * (angle_data_by_acc[i] - angle_data[i]);
      incerteza_kalman[i] = (1 - k_gain) * incerteza_kalman[i];
    }
  }
  last_millis_kalman = millis();
}

void MPUController::update() {
  //mag_update();
  accel_update();
  gyro_update();
  angle_update_kalman();
}