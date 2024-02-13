clear all;
close all;
clc;

%% Load the data
load data1.mat

Ts = t(2) - t(1)
data = iddata(y,u,Ts);

% Detrend
mu = getTrend(data,0)
data_detrended = detrend(data, mu)

nk = delayest(data_detrended)

%% ARX
orders_arx = [2,1,nk];
m_arx = arx(data_detrended, orders_arx)

%% Validation
figure; resid(m_arx, data_detrended, 'corr')
figure; iopzplot(m_arx)

%% Cross validation
load data4.mat
dataV = iddata(y,u,Ts);
dataV_detrended = detrend(dataV, getTrend(dataV,0));

k = 1;
opt = compareOptions('InitialCondition','z');
figure; compare(dataV_detrended, m_arx, k, opt)

%% Save the model
sysMotor = tf(m_arx)
save model.mat sysMotor