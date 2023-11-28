clc 
clear all
close all
%%
porta_serial = 'COM4';
baud_rate = 115200;

device = serial(porta_serial, 'BaudRate', baud_rate)
fopen(device)
%device = serialport("COM4",115200)
%write(device,1:4,"float")
%fopen(esp32_serial);
%read(esp32_serial,4,"float")
%read(device,4,"float")

%{
figure;
xlabel('Tempo');
ylabel('Ângulo');
title('Gráfico do Ângulo do ESP32');


while ishandle(1)
    dados = fscanf(esp32_serial, '%f');
   % dados = read(esp32_serial,4,"float");
    plot(dados, 'o-');
    drawnow;
end

% Fecha a porta serial ao encerrar
fclose(esp32_serial);
delete(esp32_serial);
%}