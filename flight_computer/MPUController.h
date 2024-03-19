#ifndef MPUController_h
#define MPUController_h

#include <Arduino.h>
#include <Wire.h>

class MPUController {
    public:
      MPUController();
      bool begin();
      int MPU9ADD = 0x68;
      void calibrate_mpu();
      void accel_update();
      void gyro_update();
      void update();
      void mag_update();
      float mag_data[3];
      float adjust_mag_coeff[3];
      int MAGADD;
      int verify_magnetometer();
      void angle_update_by_acc();
      void angle_update_kalman();
      float angle_data[2] = {0,0}; 
      float angle_data_by_acc[2];
      float accel_data[3], gyro_data[3];
      float a_offset[3] = { 0.0, 0.0, 0.0 };
      float g_offset[3] = { 0.0, 0.0, 0.0 };
      float mpu_resolutions[2] = {4096, 64.53};
    private:
      void calibrate_gyro();
      void calibrate_accel();
      bool verify_connection();
      long lastMillis = 0, last_millis_kalman = 0;
      int vertical_axis = 2; //Saves the vertical axis 0=x, 1=y, 2=z. Its recognized in the calibration function
      //variables para filtro de Kalman:
      float incerteza_kalman[2] = {2*2,2*2}; //x y y, respectivamente. Para el video PITCH AND ROLL.
      //X y Y, en ese orden
};

#endif