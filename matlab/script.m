clc
clear all
close all

A = [  1  0  0
       0  1  0
       0  0  1  ];
   
C = [  1  0  0
       0  1  0
       0  0  1  ];

path_data_1 = 'data/dados_antes_calibracao.csv';
path_data_2 = 'data/dados_depois_calibracao.csv';
path_data_3 = 'data/dados_estimativas_isoladas.csv';

data = readtable(path_data_3);

est_gx_by_ac = data.Var1;
est_gy_by_ac = data.Var2;
est_gz_by_ac = data.Var3;

est_gx_by_gx = data.Var4;
est_gy_by_gy = data.Var5;
est_gz_by_gz = data.Var6;

v11 = var(est_gx_by_ac);
v22 = var(est_gy_by_ac);
v33 = var(est_gz_by_ac);

w11 = var(est_gx_by_gx);
w22 = var(est_gy_by_gy);
w33 = var(est_gz_by_gz);

V = [  v11  0.0  0.0
       0.0  v22  0.0
       0.0  0.0  v33  ];
     
%W = [  w11  0.0  0.0
%       0.0  w22  0.0
%       0.0  0.0  w33  ];

W = [  1  0  0
       0  1  0
       0  0  1  ];
   
W = W'.*W;
V = V'.*V;
     
L = dlqr(A', C', W', V')';
     