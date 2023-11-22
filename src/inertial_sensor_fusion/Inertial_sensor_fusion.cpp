#include "Inertial_sensor_fusion.h"


Inertial_sensor_fusion::Inertial_sensor_fusion(){}


void Inertial_sensor_fusion::setup_kalman_filter_in_steady_state(std::vector<std::vector<float>> L){
    this->L = L;
    A = 1;
    B = 0;
    C = 1;    
}


std::vector<float> Inertial_sensor_fusion::kalman_filter_in_steady_state(float dt, std::vector<float> x, std::vector<float> u){
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

    return y_hat;
}