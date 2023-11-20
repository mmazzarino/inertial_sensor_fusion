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

data = readtable(path_data_2);

ax = data.Var1;
ay = data.Var2;
az = data.Var3;
gx = data.Var4;
gy = data.Var5;
gz = data.Var6;

V = cov([ax, ay, az]);
W = cov([gx, gy, gz]);

W = W'.*W;
V = V'.*V;

L = dlqr(A', C', W', V')';
