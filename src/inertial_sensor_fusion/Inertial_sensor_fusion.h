// Classe : InertialSensorFusion
// Descrição: Classe destinada a implementar diferentes métodos de fusão de dados de sensores inerciais
// Autor: Matheus Mazzarino
#pragma once
#include <Arduino.h>
#include <iostream>
#include <vector>


class Inertial_sensor_fusion{

    public:
        Inertial_sensor_fusion();
        void setup_kalman_filter_in_steady_state(std::vector<std::vector<float>> L);
        std::vector<float> kalman_filter_in_steady_state(float dt, std::vector<float> x, std::vector<float> u);
        
    private:
        uint8_t A;
        int16_t B;
        uint8_t C;
        uint8_t D;
        std::vector<float> y = {0.0, 0.0, 0.0};
        std::vector<float> x_hat = {0.0, 0.0, 0.0};
        std::vector<float> y_hat = {0.0, 0.0, 0.0};
        std::vector<std::vector<float>> L = {
            {0.0030, 0.0002, 0.0000},
            {0.0003, 0.0143, 0.0001},
            {0.0000, 0.0001, 0.0029}
        };
};