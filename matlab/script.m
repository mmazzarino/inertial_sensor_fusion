clc
clear all
close all

A = [  1  0  0
       0  1  0
       0  0  1  ];
   
C = [  1  0  0
       0  1  0
       0  0  1  ];

path_data = 'data/dados_ensaio_6.csv';
data = readtable(path_data);

% tempo entre ciclos (periodo de amostragem)
T = mean(data.Var7);

% estimativas de angulos de rotacao obtidas unicamente pelo acelerometro
est_gx_by_ac = data.Var1;
est_gy_by_ac = data.Var2;
est_gz_by_ac = data.Var3;

% estimativas de angulos de rotacao obtidas unicamente pelo giroscopio
est_gx_by_gx = data.Var4;
est_gy_by_gy = data.Var5;
est_gz_by_gz = data.Var6;

% variacoes das estimativas de angulos de rotacao obtidas pelo acelerometro
v11 = var(est_gx_by_ac);
v22 = var(est_gy_by_ac);
v33 = var(est_gz_by_ac);

% variacoes das estimativas de angulos de rotacao obtidas pelo giroscopio
w11 = var(est_gx_by_gx);
w22 = var(est_gy_by_gy);
w33 = var(est_gz_by_gz);

% matriz de penalizacoes do sensor (acelerometro)
V = [  v22  0.0  0.0
       0.0  v11  0.0
       0.0  0.0  v33  ];
     
W = [  w11  0.0  0.0
       0.0  w22  0.0
       0.0  0.0  w33  ];

   
W = (T^2)*W;
V = V'.*V;
     
L = dlqr(A', C', W', V')'
     