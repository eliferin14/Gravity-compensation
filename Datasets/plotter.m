clear all;
close all;
clc;

%% Load the data
data = readtable("small_step_positive.csv")

figure;
hold on;
plot(data.time, data.reference)
plot(data.time, data.theta)
grid on;
legend("Reference", "Posistion")