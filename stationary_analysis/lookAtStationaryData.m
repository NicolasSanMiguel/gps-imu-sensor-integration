clc
clear
close all



% import the data
imu1 = readtable('imu1_test3.csv');

aX = imu1.aX;
aY = imu1.aY;
aZ = imu1.aZ;


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


% stats
meanx = mean(aX);
meany = mean(aY);
meanz = mean(aZ);
stdx = std(aX);
stdy = std(aY);
stdz = std(aZ);




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


