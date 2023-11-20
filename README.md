# INERTIAL SENSOR FUSION

Este projeto foi proposto como Atividade Extra da disciplina de Sistemas de Controle Digital, do curso de Engenharia de Controle e Automação da PUCRS. A disciplina foi ministrada pelo professor Dr. Rafael da Silveira Castro, em 2023/2.

O objetivo do projeto é implementar um sistema digital de fusão de sensores inerciais de forma prática, onde os seguintes requisitos devem ser atendidos:



## Requisitos

- [x] Deve ser utilizada a metodologia "Linear Quadratic Estimation" (LQE), também chamada de "Steady-State Kalman Filter" (SSKF), conforme apresentado em aula;

- [x] O algoritmo de fusão deve ser implementado em um microcontrolador digital (Arduino, ESP32, Raspberry Pi, etc);

- [x] Deve ser utilizado um sensor de medidas inerciais (IMU) que contenha Acelerômetro e Giroscópio em 3 eixos;

- [x] O sistema deve ser capaz de reconstruir no mínimo um ângulo de orientação espacial, que tenha o eixo de rotação perpendicular ao vetor gravidade. 


Atenção: não trabalhar com a determinação de um ângulo paralelo à gravidade, pois neste caso a medição do acelerômetro será nula.



## Componentes utilizados

- [x] ESP32 TTGO T-Beam T22_V1.1
- [x] GY-91 (MPU9250 + BMP280)



## Ambiente de desenvolvimento

O projeto de software foi desenvolvido utilizando o Visual Studio Code, com a extensão PlatformIO IDE, sendo que PlataformIO foi configurado para utilizar o framework Arduino. 

Os arquivos relacionados ao Matlab foram desenvolvidos utilizando o Matlab R2017b.



## Comentários adicionais

Este repositório contém todos arquivos utilizados no desenvolvimento do projeto, incluindo o código fonte, arquivos Matlab e o relatório. 

O projeto presente neste repositório poderá seguir em desenvolvimento, recebendo melhorias e novas funcionalidades, após a entrega do mesmo.



