#include <Arduino.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "Wire.h"
#include "external/ESP32_Arduino_MPU9250/MPU9250.h"
#include "inertial_sensor_fusion/Inertial_sensor_fusion.h"
#include <unistd.h>

void printing_raw_data_on_the_serial();
void calibrate_accel();
void calibrate_gyro();
void calibrate_mag();

unsigned long last_time;
unsigned long now;
float dt;
float mx;
float my;
float mz;
float temp;

std::vector<float> acel = {0.0, 0.0, 0.0};
std::vector<float> giro = {0.0, 0.0, 0.0};
std::vector<float> mag = {0.0, 0.0, 0.0};
std::vector<float> orientacao = {0.0, 0.0, 0.0};
std::vector<float> acc_orientation_est = {0.0, 0.0, 0.0};
std::vector<float> gyr_orientation_est = {0.0, 0.0, 0.0};

// Matriz L obtida no Matlab a partir dos dados do ensaio
std::vector<std::vector<float>> L = {
  {0.9112, 0.0000, 0.0000},
  {0.0000, 0.6154, 0.0000},
  {0.0000, 0.0000, 0.0006}
};

MPU9250 IMU(i2c0, 0x68);
Inertial_sensor_fusion ISF;

void setup() {
  Serial.begin(115200);
  if (IMU.begin() < 0){
    Serial.println("IMU initialization unsuccessful");
    while (1){}
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  calibrate_accel();
  calibrate_gyro();
  calibrate_mag();
  ISF.setup_steady_state_kalman_filter(L);
  last_time = millis();
}


void loop() {
  now = millis();
  dt = (now - last_time) / 1000.0;
  last_time = now;
  
  IMU.readSensor();
  acel[0] = IMU.getAccelX_mss();
  acel[1] = IMU.getAccelY_mss();
  acel[2] = IMU.getAccelZ_mss();
  giro[0] = IMU.getGyroX_rads();
  giro[1] = IMU.getGyroY_rads();
  giro[2] = IMU.getGyroZ_rads();
  mag[0] = IMU.getMagX_uT();
  mag[1] = IMU.getMagY_uT();
  mag[2] = IMU.getMagZ_uT();
  temp = IMU.getTemperature_C();

  orientacao = ISF.steady_state_kalman_filter(dt, acel, giro);
  Serial.printf("%.2f;%.2f;%.2f;%.4f", orientacao[0], orientacao[1], orientacao[2], dt);
  Serial.println();

  //// UTILIZADO PARA O ENSAIO (OBTENÇÃO DA MATRIZ L NO MATLAB)
  // acc_orientation_est = ISF.accelerometer_orientation_estimation(acel);
  // gyr_orientation_est = ISF.gyroscope_orientation_estimation(dt, giro);
  // printing_raw_data_on_the_serial();
  delay(20);
}


void printing_raw_data_on_the_serial(){
  // imprime os dados separados por ";" para criar um arquivo csv para realizar os ensaios no o matlab
  Serial.printf("%.5f;%.5f;%.5f;%.5f;%.5f;%.5f;%.5f", acc_orientation_est[0], acc_orientation_est[1], acc_orientation_est[2], gyr_orientation_est[0], gyr_orientation_est[1], gyr_orientation_est[2], dt);
  Serial.println();
}


void calibrate_accel() {
  float axb, azb, ayb;
  float axs, ays, azs;
  if (IMU.calibrateAccel() < 0) {
    Serial.println("Failed to calibrate accelerometer");
    while (1){}
  }
  axb = IMU.getAccelBiasX_mss();
  ayb = IMU.getAccelBiasY_mss();
  azb = IMU.getAccelBiasZ_mss();
  axs = IMU.getAccelScaleFactorX();
  ays = IMU.getAccelScaleFactorY();
  azs = IMU.getAccelScaleFactorZ();
  IMU.setAccelCalX(axb,axs);
  IMU.setAccelCalY(ayb,ays);
  IMU.setAccelCalZ(azb,azs);
}


void calibrate_gyro() {
  float gxb, gyb, gzb;
  if (IMU.calibrateGyro() < 0) {
    Serial.println("Failed to calibrate gyroscope");
    while (1){}
  }
  gxb = IMU.getGyroBiasX_rads();
  gyb = IMU.getGyroBiasY_rads();
  gzb = IMU.getGyroBiasZ_rads();
  IMU.setGyroBiasX_rads(gxb);
  IMU.setGyroBiasY_rads(gyb);
  IMU.setGyroBiasZ_rads(gzb);
}


void calibrate_mag() {
  float hxb, hyb, hzb;
  float hxs, hys, hzs;
  if (IMU.calibrateMag() < 0) {
    Serial.println("Failed to calibrate magnetometer");
    while (1){}
  } 
  hxb = IMU.getMagBiasX_uT();
  hyb = IMU.getMagBiasY_uT();
  hzb = IMU.getMagBiasZ_uT();
  hxs = IMU.getMagScaleFactorX();
  hys = IMU.getMagScaleFactorY();
  hzs = IMU.getMagScaleFactorZ();
  IMU.setMagCalX(hxb,hxs);
  IMU.setMagCalY(hyb,hys);
  IMU.setMagCalZ(hzb,hzs);
}
