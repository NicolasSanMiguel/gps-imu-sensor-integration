clc
clear
close all

% % % AA 272 Project Code - Winter 2022
% % % Data taken on 20 February 2022

% % STATIONARY
% % Read IMU data
% IMU_1 = readtable('run1.csv');
% IMU_2 = readtable('run1.csv');
% IMU_3 = readtable('run1.csv');
% 
% % Read GPS data
% GPS_1 = readtable('SF_run_1_GPS.csv');
% GPS_2 = readtable('SF_run_1_GPS.csv');
% GPS_3 = readtable('SF_run_1_GPS.csv');


% WALKING MANZANITA
% Read IMU & GPS data
% IMU_walk = readtable('imu_data_walking_manzanita.csv');
IMU_walk = readtable('imu_data_walking_manzanita.csv');
GPS_walk = readtable('walking_manzanita_GPS.csv');


% smooth the data
aX = IMU_walk.aX;
aY = IMU_walk.aY;
aZ = IMU_walk.aZ;

aX = smoothdata(aX);
aY = smoothdata(aY);
aZ = smoothdata(aZ);
aX = smoothdata(aX);
aY = smoothdata(aY);
aZ = smoothdata(aZ);

% normalize the accelerometer data
% based on the mean values of the stationary data tests
aX = (aX./16547)*9.81; % m/s2
aY = (aY./16547)*9.81; % m/s2
aZ = (aZ./16547)*9.81; % m/s2

%      Directions      Means      Stds 
%     _____________    ______    ______
%     "x-direction"     2335     45.65
%     "y-direction"     68.53    29.43
%     "z-direction"     16547    45.07

% numerical integration of IMU accelerometer data
% v = v0 + at
vX = accel2vel(aX);
vY = accel2vel(aY);
vZ = accel2vel(aZ);


figure
xvals = 1:length(vX);
xvals = xvals./10;
subplot(1,3,1)
plot(xvals,aX)
grid on
xlabel('time [sec]'); ylabel('x-accel [units/s^2]')

subplot(1,3,2)
plot(xvals,aY)
grid on
xlabel('time [sec]'); ylabel('y-accel [units/s^2]')

subplot(1,3,3)
plot(xvals,aZ)
grid on
xlabel('time [sec]'); ylabel('z-accel [units/s^2]')




figure
xvals = 1:length(vX);
xvals = xvals./10;
subplot(1,3,1)
plot(xvals,vX)
grid on
xlabel('time [sec]'); ylabel('x-velocity [units/s]')

subplot(1,3,2)
plot(xvals,vY)
grid on
xlabel('time [sec]'); ylabel('y-velocity [units/s]')

subplot(1,3,3)
plot(xvals,vZ)
grid on
xlabel('time [sec]'); ylabel('z-velocity [units/s]')




% Process GPS data
% GPSx = GPS_walk.x;
% GPSy = GPS_walk.y;
% GPSz = GPS_walk.z;
XYZ = lla2ecef([GPS_walk.lat, GPS_walk.long, GPS_walk.alt]);
GPSx = XYZ(:, 1);
GPSy = XYZ(:, 2);
GPSz = XYZ(:, 3);




close all
%%
% Handle IMU/GPS length mismatch
dtIMU = 0.1; % sec
% dtGPS = GPS_walk.dt(1); % sec
dtGPS = 1; % sec

timeIMU = 0:dtIMU:dtIMU*(length(aX)-1);
timeGPS = 0:dtGPS:dtGPS*(length(GPSx)-1);
% cut the longer off at the shorter
% timeIMU(end) = 202.9
% timeGPS(end) = 228
cuttime1 = floor(timeIMU(end)); % should be 202
cuttime2 = floor(timeGPS(end)); % should be 202
if cuttime1 < cuttime2 % choose the shorter cuttime
    cuttime = cuttime1;
else
    cuttime = cuttime2;
end
timeIMU = timeIMU(timeIMU<=cuttime);
timeGPS = timeGPS(timeGPS<=cuttime);

aX = aX(1:length(timeIMU));
aY = aY(1:length(timeIMU));
aZ = aZ(1:length(timeIMU));
GPSx = GPSx(1:length(timeGPS));
GPSy = GPSy(1:length(timeGPS));
GPSz = GPSz(1:length(timeGPS));

meanx = mean(aX);
meany = mean(aY);
meanz = mean(aZ);
stdx = std(aX)
stdy = std(aY)
stdz = std(aZ)




%{
% Interpolate GPS data
% aX, aY, and aZ are 2030x1 while GPSx, y, z are all 229x1 [walking manzanita data]
% so, we interpolate the GPS data so it is the same length, since each
% series was recorded over the same time interval
xq = 0:0.1:xvals(end);
newxvals = linspace(0,xvals(end),length(GPSx));
GPSx2 = interp1(newxvals,GPSx,xq);
GPSx2 = GPSx2(1:end-1);

yq = 0:0.1:xvals(end);
newxvals = linspace(0,xvals(end),length(GPSy));
GPSy2 = interp1(newxvals,GPSy,yq);
GPSy2 = GPSy2(1:end-1);

zq = 0:0.1:xvals(end);
newxvals = linspace(0,xvals(end),length(GPSz));
GPSz2 = interp1(newxvals,GPSz,zq);
GPSz2 = GPSz2(1:end-1);
%}







figure
plot3(GPSx,GPSy,GPSz)
xlabel('x-position [m]'); ylabel('y-position [m]'); zlabel('z-position [m]')
grid on
hold on
% plot3(GPSx2,GPSy2,GPSz2)


function vdata = accel2vel(aXYZ)
% numerical interpolation of acceleration data, assuming an offset of v0=0
vX = [];
delta_t = 0.1; % seconds
for i = 1:length(aXYZ)
    vX(i) = delta_t*aXYZ(i);
end
vdata = vX;

end



