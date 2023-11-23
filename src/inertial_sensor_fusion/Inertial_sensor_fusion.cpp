#include "Inertial_sensor_fusion.h"


Inertial_sensor_fusion::Inertial_sensor_fusion(){}


std::vector<float> Inertial_sensor_fusion::accelerometer_orientation_estimation(std::vector<float> acel){
    // Estima a orientação do acelerômetro para ensaio e obtenção da matriz L    
    // Tait-Bryan angles
    acc_orientation_est[0] = (atan2( acel[1], sqrt(pow(acel[0], 2) + pow(acel[2], 2)))) * RAD_TO_DEG;
    acc_orientation_est[1] = (atan2(-acel[0], sqrt(pow(acel[1], 2) + pow(acel[2], 2)))) * RAD_TO_DEG;
    acc_orientation_est[2] = (atan2( acel[2], sqrt(pow(acel[0], 2) + pow(acel[1], 2)))) * RAD_TO_DEG;

    return acc_orientation_est;
}


std::vector<float> Inertial_sensor_fusion::gyroscope_orientation_estimation(float dt, std::vector<float> giro){
    // Estima a orientação do giroscópio para ensaio e obtenção da matriz L
    // Obs.: atualmente a matriz L é obtida considerando W = eye(3)

    gyr_orientation_est[0] = gyr_orientation_est[0] + giro[0]*dt;
    gyr_orientation_est[1] = gyr_orientation_est[1] + giro[1]*dt;
    gyr_orientation_est[2] = gyr_orientation_est[2] + giro[2]*dt;

    return gyr_orientation_est;
}


void Inertial_sensor_fusion::setup_steady_state_kalman_filter(std::vector<std::vector<float>> L){
    this->L = L;
    A = 1;
    B = 0;
    C = 1;
}


std::vector<float> Inertial_sensor_fusion::steady_state_kalman_filter(float dt, std::vector<float> x, std::vector<float> u){
    // Linear Quadratic Estimation (LQE), também chamada de Steady-State Kalman Filter (SSKF)
    // x_hat' = A*x_hat + B*u + L*(y - C*x_hat)
    // y_hat = C*x_hat + D*u
    
    B = dt;
    // Tait-Bryan angles
    y[0] = (atan2( x[1], sqrt(pow(x[0], 2) + pow(x[2], 2)))) * RAD_TO_DEG;
    y[1] = (atan2(-x[0], sqrt(pow(x[1], 2) + pow(x[2], 2)))) * RAD_TO_DEG;
    y[2] = (atan2( x[2], sqrt(pow(x[0], 2) + pow(x[1], 2)))) * RAD_TO_DEG;

    x_hat[0] = A*x_hat[0] + B*u[0] + L[0][0]*(y[0] - C*x_hat[0]);
    x_hat[1] = A*x_hat[1] + B*u[1] + L[1][1]*(y[1] - C*x_hat[1]);
    x_hat[2] = A*x_hat[2] + B*u[2] + L[2][2]*(y[2] - C*x_hat[2]);

    y_hat[0] = C*x_hat[0];
    y_hat[1] = C*x_hat[1];
    y_hat[2] = C*x_hat[2];

    return y_hat;
}