clc
clear
close all



% import the data
imu1 = readtable('imu1_test1.csv');

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

exit
% plotting
f = figure;  
f.Position = [100 100 1200 400]; 
title('Collection 1')

subplot(1,3,1)
plot(xvals2plot(1:end-1),aX)
title('x-values over time')
grid on; hold on
plot(xvals2plot(1:end-1),meanx*ones(1,length(xvals2plot(1:end-1))),'--','LineWidth',2)
txt = {'Standard deviation',num2str(stdx)};
text(xvals2plot(100),2150,txt)

subplot(1,3,2)
plot(xvals2plot(1:end-1),aY)
title('y-values over time')
grid on; hold on
plot(xvals2plot(1:end-1),meany*ones(1,length(xvals2plot(1:end-1))),'--','LineWidth',2)
txt = {'Standard deviation',num2str(stdy)};
text(xvals2plot(10),0,txt)

subplot(1,3,3)
plot(xvals2plot(1:end-1),aZ)
title('z-values over time')
grid on; hold on
plot(xvals2plot(1:end-1),meanz*ones(1,length(xvals2plot(1:end-1))),'-','LineWidth',2)
txt = {'Standard deviation',num2str(stdz)};
text(xvals2plot(10),16450,txt)

% close all
% make a table to output information
Directions = ["x-direction";"y-direction";"z-direction"];
Means = [meanx; meany; meanz];
Stds = [stdx; stdy; stdz];
ourData = table(Directions,Means,Stds)




