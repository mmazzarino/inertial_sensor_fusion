#include "Inertial_sensor_fusion.h"


Inertial_sensor_fusion::Inertial_sensor_fusion(){}


void Inertial_sensor_fusion::setup_steady_state_kalman_filter(std::vector<std::vector<float>> L){
    this->L = L;
    A = 1;
    B = 0;
    C = 1;
}


std::vector<float> Inertial_sensor_fusion::accelerometer_orientation_estimation(std::vector<float> acel){
    // método utilizado para ensaio e posterior obtenção da matriz L
    acc_orientation_est[0] = (atan2(-acel[0], -acel[2]));
    acc_orientation_est[1] = (atan2(-acel[1], -acel[2]));
    acc_orientation_est[2] = (atan2(-acel[0], -acel[1]));
    return acc_orientation_est;
}


std::vector<float> Inertial_sensor_fusion::gyroscope_orientation_estimation(float dt, std::vector<float> giro){
    // Método utilizado para ensaio e posterior obtenção da matriz L
    gyr_orientation_est[0] = gyr_orientation_est[0] + (giro[0] * dt);
    gyr_orientation_est[1] = gyr_orientation_est[1] + (giro[1] * dt);
    gyr_orientation_est[2] = gyr_orientation_est[2] + (giro[2] * dt);
    return gyr_orientation_est;
}


std::vector<float> Inertial_sensor_fusion::steady_state_kalman_filter(float dt, std::vector<float> x, std::vector<float> u){
    // Linear Quadratic Estimation (LQE), também chamada de Steady-State Kalman Filter (SSKF)
    B = dt;

    y[0] = (atan2(-x[0], -x[2]));
    y[1] = (atan2(-x[1], -x[2]));
    y[2] = (atan2(-x[0], -x[1]));

    x_hat[0] = A*x_hat[0] + B*u[0] + L[0][0]*(y[0] - C*x_hat[0]);
    x_hat[1] = A*x_hat[1] + B*u[1] + L[1][1]*(y[1] - C*x_hat[1]);
    x_hat[2] = A*x_hat[2] + B*u[2] + L[2][2]*(y[2] - C*x_hat[2]);

    y_hat[0] = C*x_hat[0] * RAD_TO_DEG;
    y_hat[1] = C*x_hat[1] * RAD_TO_DEG;
    y_hat[2] = C*x_hat[2] * RAD_TO_DEG;

    return y_hat;
}