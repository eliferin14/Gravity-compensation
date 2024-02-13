data = data(1:73,:);

t = data(:,1)
u = data(:,2)
dutycycle = data(:,3)
y = data(:,4)

close all;
figure; hold on;
plot(t,u);
plot(t,dutycycle);
plot(t,y);
grid on;
legend("Sinusoidal", "Dutycycle", "Angle")

save('data4.mat', 't', 'u', "y")