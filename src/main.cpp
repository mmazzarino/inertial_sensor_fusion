#include <Arduino.h>
#include <iostream>
#include "Wire.h"
#include "external/ESP32_Arduino_MPU9250/MPU9250.h"


MPU9250 IMU(i2c0, 0x68);
int status;
unsigned long last_time;
unsigned long now;
float dt;
float mx;
float my;
float mz;
float temp;

int16_t B = 0;

uint8_t A = 1;

uint8_t C = 1;

float x_hat[3] = {
    0.0,
    0.0,
    0.0
};

float y_hat[3] = {
    0.0,
    0.0,
    0.0
};

float u[3] = {
    0.0,            // gx
    0.0,            // gy
    0.0             // gz
}; 

float x[3] = {
    0.0,            // ax
    0.0,            // ay
    0.0             // az
};

float y[3] = {
    0.0,
    0.0,
    0.0
};

// gerado no matlab a partir dos dados obtidos pós calibração
float L[3][3] = {
    {0.0030, 0.0002, 0.0000},
    {0.0003, 0.0143, 0.0001},
    {0.0000, 0.0001, 0.0029}
};

void print_header_serial_in_csv_format();

void printing_raw_data_on_the_serial();

void kalman_filter_in_steady_state(float dt);

void calibrate_accel();

void calibrate_gyro();

void calibrate_mag();

void setup() {
  Serial.begin(9600);
  
  status = IMU.begin();
  if (status < 0)
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_16G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);

  // calibrate_accel();
  // calibrate_gyro();
  // calibrate_mag();

  last_time = millis();
}


void loop() {
  now = millis();
  dt = (now - last_time) / 1000.0;
  last_time = now;
  
  IMU.readSensor();
  
  x[0] = IMU.getAccelX_mss();
  x[1] = IMU.getAccelY_mss();
  x[2] = IMU.getAccelZ_mss();

  u[0] = IMU.getGyroX_rads();
  u[1] = IMU.getGyroY_rads();
  u[2] = IMU.getGyroZ_rads();

  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();

  temp = IMU.getTemperature_C();
  // printing_raw_data_on_the_serial();
  kalman_filter_in_steady_state(dt);
  delay(5);
}


void kalman_filter_in_steady_state(float dt) {
    B = dt;

    y[0] = (atan2( x[1], sqrt(pow(x[0], 2) + pow(x[2], 2)))) * RAD_TO_DEG;
    y[1] = (atan2(-x[0], sqrt(pow(x[1], 2) + pow(x[2], 2)))) * RAD_TO_DEG;
    y[2] = (atan2( x[2], sqrt(pow(x[0], 2) + pow(x[1], 2)))) * RAD_TO_DEG;

    x_hat[0] = A*x_hat[0] + B*u[0] + L[0][0]*(y[0] - C*x_hat[0]);
    x_hat[1] = A*x_hat[1] + B*u[1] + L[1][1]*(y[1] - C*x_hat[1]);
    x_hat[2] = A*x_hat[2] + B*u[2] + L[2][2]*(y[2] - C*x_hat[2]);

    y_hat[0] = C*x_hat[0];
    y_hat[1] = C*x_hat[1];
    y_hat[2] = C*x_hat[2];

    Serial.printf("Giro X: %.1f°  -  Giro Y: %.1f°  -  Giro Z: %.1f°\n", y_hat[0], y_hat[1], y_hat[2]);
    Serial.println();
}


void printing_raw_data_on_the_serial(){
  // imprime os dados separados por ";" para criar um arquivo csv para o matlab
  Serial.print(x[0]); Serial.print(";"); Serial.print(x[1]); Serial.print(";"); Serial.print(x[2]); Serial.print(";");
  Serial.print(u[0]); Serial.print(";"); Serial.print(u[1]); Serial.print(";"); Serial.print(u[2]); Serial.print(";");
  Serial.print(dt); Serial.print(";");
  Serial.println();
}


void calibrate_accel() {
  float axb, azb, ayb;
  float axs, ays, azs;
  status = IMU.calibrateAccel();
  if (status < 0) {
    Serial.println("Failed to calibrate accelerometer");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
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
  status = IMU.calibrateGyro();
  if (status < 0) {
    Serial.println("Failed to calibrate gyroscope");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
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
  status = IMU.calibrateMag();
  if (status < 0) {
    Serial.println("Failed to calibrate magnetometer");
    Serial.print("Status: ");
    Serial.println(status);
    while (1)
    {
    }
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
