clc
clear
close all

plotAccHistos = false;
plotAllanVarACC = true;
plotAllanVar = false;
% import the data
imu1 = readtable('imu1_test3.csv');

aX = imu1.aX;
aY = imu1.aY;
aZ = imu1.aZ;

gX = imu1.gX;
gY = imu1.gY;
gZ = imu1.gZ;

% timestamps = imu1.timestamp;
% timestep = timestamps(2) - timestamps(1);
timestep = 0.1; % seconds
xvals2plot = [0:timestep:timestep*(length(aX)-1)];

% % smooth the data
aX = smoothdata(aX);
aY = smoothdata(aY);
aZ = smoothdata(aZ);
aX = smoothdata(aX);
aY = smoothdata(aY);
aZ = smoothdata(aZ);

gX = smoothdata(gX);
gY = smoothdata(gY);
gZ = smoothdata(gZ);
gX = smoothdata(gX);
gY = smoothdata(gY);
gZ = smoothdata(gZ);

% stats - accelerometer
meanx = mean(aX);
meany = mean(aY);
meanz = mean(aZ);
stdx = std(aX);
stdy = std(aY);
stdz = std(aZ);

% normalize the accelerometer data
% based on the mean values of the stationary data tests
aX = (aX./16547)*9.81; % m/s2
aY = (aY./16547)*9.81; % m/s2
aZ = (aZ./16547)*9.81; % m/s2


if plotAccHistos
    f = figure;
    f.Position = [100 100 1200 400];

    subplot(1,3,1)
    histogram(aX); hold on; ylabel('Count'); yyaxis right
    xvals = [1.25:0.001:1.45];
    y_norm = normpdf(xvals,mean(aX),std(aX));
    plot(xvals,y_norm,'LineWidth',2)
    % pd = fitdist(aX,'Normal')
    xlabel('Measured acceleration [m/s^2]'); ylabel('Probability density')
    title('Stationary data in x-direction')

    subplot(1,3,2)
    histogram(aY); hold on; ylabel('Count'); yyaxis right
    xvals = [0:0.001:0.09];
    y_norm = normpdf(xvals,mean(aY),std(aY));
    plot(xvals,y_norm,'LineWidth',2)
    % pd = fitdist(aX,'Normal')
    xlabel('Measured acceleration [m/s^2]'); ylabel('Probability density')
    title('Stationary data in y-direction')

    subplot(1,3,3)
    histogram(aZ); hold on; ylabel('Count'); yyaxis right
    xvals = [9.74:0.001:9.95];
    y_norm = normpdf(xvals,mean(aZ),std(aZ));
    plot(xvals,y_norm,'LineWidth',2)
    % pd = fitdist(aX,'Normal')
    xlabel('Measured acceleration [m/s^2]'); ylabel('Probability density')
    title('Stationary data in z-direction')
end

gX = (gX/250);
gY = gY/250;
gZ = gZ/250;
% gX = gX/mean(gX);
% gY = gY/mean(gY);
% gZ = gZ/mean(gZ);

% stats - gyro
meanx = mean(gX);
meany = mean(gY);
meanz = mean(gZ);
stdx = std(gX);
stdy = std(gY);
stdz = std(gZ);

% make a table to output information
Directions = ["x-direction";"y-direction";"z-direction"];
Means = [meanx; meany; meanz];
Stds = [stdx; stdy; stdz];
ourData = table(Directions,Means,Stds)


% vgX = accel2vel(gX);
% vgY = accel2vel(gY);
% vgZ = accel2vel(gZ);


if plotAllanVarACC
    Fs = 1;
    [avar,tau] = allanvar(aX,'octave',Fs);
    loglog(tau,avar)
    hold on
    [avar,tau] = allanvar(aY,'octave',Fs);
    loglog(tau,avar)
    [avar,tau] = allanvar(aZ,'octave',Fs);
    loglog(tau,avar)

    xlabel('\tau [seconds]')
    ylabel('\sigma^2(\tau) [\circ/sec]')
    title('Allan Variance')
    legend('aX','aY','aZ')
    grid on
end


if plotAllanVar
    Fs = 1;
    [avar,tau] = allanvar(gX,'octave',Fs);
    loglog(tau,avar)
    hold on
    [avar,tau] = allanvar(gY,'octave',Fs);
    loglog(tau,avar)
    [avar,tau] = allanvar(gZ,'octave',Fs);
    loglog(tau,avar)

    xlabel('\tau [seconds]')
    ylabel('\sigma^2(\tau) [\circ/sec]')
    title('Allan Variance')
    legend('gX','gY','gZ')
    grid on
end
%{
% plotting
f = figure;  
f.Position = [100 100 1200 400]; 
title('Collection 1')

subplot(1,3,1)
plot(xvals2plot,aX)
title('x-values over time')
grid on; hold on
plot(xvals2plot,meanx*ones(1,length(xvals2plot)),'--','LineWidth',2)
txt = {'Standard deviation',num2str(stdx)};
text(xvals2plot(100),2150,txt)

subplot(1,3,2)
plot(xvals2plot,aY)
title('y-values over time')
grid on; hold on
plot(xvals2plot,meany*ones(1,length(xvals2plot)),'--','LineWidth',2)
txt = {'Standard deviation',num2str(stdy)};
text(xvals2plot(10),0,txt)

subplot(1,3,3)
plot(xvals2plot,aZ)
title('z-values over time')
grid on; hold on
plot(xvals2plot,meanz*ones(1,length(xvals2plot)),'--','LineWidth',2)
txt = {'Standard deviation',num2str(stdz)};
text(xvals2plot(10),16450,txt)

% close all
% make a table to output information
Directions = ["x-direction";"y-direction";"z-direction"];
Means = [meanx; meany; meanz];
Stds = [stdx; stdy; stdz];
ourData = table(Directions,Means,Stds)
%}




%{
THESE THREE TABLES WERE SMOOTHED TWICE
Stationary test 1
     Directions      Means      Stds 
    _____________    ______    ______

    "x-direction"      2391    55.893
    "y-direction"    79.649     30.74
    "z-direction"     16560    46.449

Stationary test 2
     Directions      Means      Stds 
    _____________    ______    ______

    "x-direction"    2327.4    33.619
    "y-direction"    55.861    28.787
    "z-direction"     16503    42.804

Stationary test 3
     Directions      Means      Stds 
    _____________    ______    ______

    "x-direction"    2287.8    47.445
    "y-direction"     70.08    28.769
    "z-direction"     16580    45.965


-------------------------------------------
Summary of all 3
     Directions      Means      Stds 
    _____________    ______    ______
    "x-direction"     2335     45.65
    "y-direction"     68.53    29.43
    "z-direction"     16547    45.07

%}

function vdata = accel2vel(aXYZ)
% numerical interpolation of acceleration data, assuming an offset of v0=0
vX = [];
delta_t = 0.1; % seconds
for i = 1:length(aXYZ)
    vX(i) = delta_t*aXYZ(i);
end
vdata = vX;
end

