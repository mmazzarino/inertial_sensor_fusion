#include <Arduino.h>
#include <iostream>
#include <vector>
#include "Wire.h"
#include "external/ESP32_Arduino_MPU9250/MPU9250.h"
#include "inertial_sensor_fusion/Inertial_sensor_fusion.h"


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
std::vector<float> orientacao = {0.0, 0.0, 0.0};
std::vector<std::vector<float>> L = {
  {0.0030, 0.0002, 0.0000},
  {0.0003, 0.0143, 0.0001},
  {0.0000, 0.0001, 0.0029}
};

MPU9250 IMU(i2c0, 0x68);
Inertial_sensor_fusion ISF;


void setup() {
  Serial.begin(9600);
  if (IMU.begin() < 0){
    Serial.println("IMU initialization unsuccessful");
    while (1){}
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  // calibrate_accel();
  // calibrate_gyro();
  // calibrate_mag();
  ISF.setup_kalman_filter_in_steady_state(L);
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
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  temp = IMU.getTemperature_C();
  // printing_raw_data_on_the_serial();
  
  orientacao = ISF.kalman_filter_in_steady_state(dt, acel, giro);
  Serial.printf("Giro X: %.1f°  -  Giro Y: %.1f°  -  Giro Z: %.1f°\n", orientacao[0], orientacao[1], orientacao[2]);
  Serial.println();
  
  delay(5);
}


void printing_raw_data_on_the_serial(){
  // imprime os dados separados por ";" para criar um arquivo csv para o matlab
  Serial.print(acel[0]); Serial.print(";"); Serial.print(acel[1]); Serial.print(";"); Serial.print(acel[2]); Serial.print(";");
  Serial.print(giro[0]); Serial.print(";"); Serial.print(giro[1]); Serial.print(";"); Serial.print(giro[2]); Serial.print(";");
  Serial.print(dt); Serial.print(";");
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
