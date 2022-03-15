clc
clear
close all

% % AA 272 Project Code - Winter 2022
% Custom filter

load('all_our_data.mat');




% smooth the data
aX = IMU_3.aX;
aY = IMU_3.aY;
aZ = IMU_3.aZ;

aX = smoothdata(aX);
aY = smoothdata(aY);
aZ = smoothdata(aZ);
aX = smoothdata(aX);
aY = smoothdata(aY);
aZ = smoothdata(aZ);

% normalize the accelerometer data
% based on the mean values of the stationary data tests
aX = (aX./mean(aX))-1;
aY = (aY./mean(aY))-1;
aZ = (aZ./mean(aZ))-1;

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

% 
% figure
% xvals = 1:length(vX);
% xvals = xvals./10;
% subplot(1,3,1)
% plot(xvals,aX)
% grid on
% xlabel('time [sec]'); ylabel('x-accel [units/s^2]')
% 
% subplot(1,3,2)
% plot(xvals,aY)
% grid on
% xlabel('time [sec]'); ylabel('y-accel [units/s^2]')
% 
% subplot(1,3,3)
% plot(xvals,aZ)
% grid on
% xlabel('time [sec]'); ylabel('z-accel [units/s^2]')

% figure
% xvals = 1:length(vX);
% xvals = xvals./10;
% subplot(1,3,1)
% plot(xvals,vX)
% grid on
% xlabel('time [sec]'); ylabel('x-velocity [units/s]')
% 
% subplot(1,3,2)
% plot(xvals,vY)
% grid on
% xlabel('time [sec]'); ylabel('y-velocity [units/s]')
% 
% subplot(1,3,3)
% plot(xvals,vZ)
% grid on
% xlabel('time [sec]'); ylabel('z-velocity [units/s]')


% Process GPS data
GPSx = GPS_3.x;
GPSy = GPS_3.y;
GPSz = GPS_3.z;

% Interpolate GPS data
% aX, aY, and aZ are 2030x1 while GPSx, y, z are all 229x1 [walking manzanita data]
% so, we interpolate the GPS data so it is the same length, since each
% series was recorded over the same time interval
xvals = 1:length(vX);
xq = 0:1:xvals(end);
newxvals = linspace(0,xvals(end),length(GPSx));
GPSx2 = interp1(newxvals,GPSx,xq);
GPSx2 = GPSx2(1:end-1);

yq = 0:1:xvals(end);
newxvals = linspace(0,xvals(end),length(GPSy));
GPSy2 = interp1(newxvals,GPSy,yq);
GPSy2 = GPSy2(1:end-1);

zq = 0:1:xvals(end);
newxvals = linspace(0,xvals(end),length(GPSz));
GPSz2 = interp1(newxvals,GPSz,zq);
GPSz2 = GPSz2(1:end-1);

% figure
% plot3(GPSx,GPSy,GPSz)
% xlabel('x-position [m]'); ylabel('y-position [m]'); zlabel('z-position [m]')
% grid on
% hold on
% plot3(GPSx2,GPSy2,GPSz2)
























% use the interpolated GPS data
GPSx = GPSx2;
GPSy = GPSy2;
GPSz = GPSz2;

% initial GPS position
pos_i = [GPSx(1); GPSy(1); GPSz(1)];

% % % % % % % % % IMU only track 
% this loop is separate because pos_curr must be different, and i didn't
% want to have two different pos_curr's in one loop
pos_curr = pos_i;
imu_only_positions = [];
for i = 1:length(GPSx) % estimate subsequent positions based on GPS+IMU
    nextIMU = update_posIMU(pos_curr, [vX(i); vY(i); vZ(i)]); % next point only relying on IMU
    imu_only_positions(:,i) = nextIMU;
    pos_curr = nextIMU;
end

% % % % % % % % % IMU + GPS Filter 
pos_curr = pos_i;
filtered_positions = [];
n = 100; % number of next GPS steps to measure

outarr = [];
w_IMUarr = [0, 0.5, 0.75, 0.9, 0.99, 1.0];
for j = 1:length(w_IMUarr)
    w_IMU = w_IMUarr(j);
% w_IMU = 0.5; % relative weight of IMU data
w_GPS = 1 - w_IMU; % relative weight of GPS data (w_IMU + w_GPS = 1)
for i = 1:length(GPSx)-n % estimate subsequent positions based on GPS+IMU
    nextGPS = [GPSx(i+1:i+n); GPSy(i+1:i+n); GPSz(i+1:i+n)]; % mean of next n GPS measurements
    nextGPS = [mean(nextGPS(1,:)); mean(nextGPS(2,:)); mean(nextGPS(3,:))];

    nextIMU = update_posIMU(pos_curr, [vX(i); vY(i); vZ(i)]); % next point only relying on IMU

    midpoint = [(w_GPS*nextGPS(1) + w_IMU*nextIMU(1)); 
                (w_GPS*nextGPS(2) + w_IMU*nextIMU(2)); 
                (w_GPS*nextGPS(3) + w_IMU*nextIMU(3))];
    filtered_positions(:,i) = midpoint;
    pos_curr = midpoint;
end
outarr(:,:,j) = filtered_positions(:,:);
end

%% Plotting
% 2D plot with different weights
figure
plot(GPSx, GPSy)
hold on; grid on;
plot(imu_only_positions(1,:), imu_only_positions(2,:),'LineWidth',2)

plot(outarr(1,:,1), outarr(2,:,1),'LineWidth',2)
% plot(outarr(1,:,2), outarr(2,:,2),'LineWidth',2)
% plot(outarr(1,:,3), outarr(2,:,3),'LineWidth',2)
plot(outarr(1,:,4), outarr(2,:,4),'LineWidth',2)
% plot(outarr(1,:,5), outarr(2,:,5),'LineWidth',2)
plot(outarr(1,:,6), outarr(2,:,6),'LineWidth',2)

plot(pos_i(1), pos_i(2),'gx','LineWidth',2)
legend('GPS only','IMU only','GPS + IMU, wIMU = 0',...%     'GPS + IMU, wIMU = 0.5','GPS + IMU, wIMU = 0.75',...   
            'GPS + IMU, wIMU = 0.9',... %'GPS + IMU, wIMU = 0.99',...
    'GPS + IMU, wIMU = 1.0','Starting Point')
xlabel('x-position'); ylabel('y-position')


% % 2D plot
% figure
% plot(GPSx, GPSy)
% hold on; grid on;
% plot(imu_only_positions(1,:), imu_only_positions(2,:),'LineWidth',2)
% 
% plot(filtered_positions(1,:), filtered_positions(2,:),'LineWidth',2)
% plot(pos_i(1), pos_i(2),'gx','LineWidth',2)
% legend('GPS only','IMU only','GPS + IMU','Starting Point')
% xlabel('x-position'); ylabel('y-position')

% % 3D plot
% figure
% plot3(GPSx, GPSy, GPSz)
% hold on; grid on;
% plot3(filtered_positions(1,:), filtered_positions(2,:), filtered_positions(3,:),'LineWidth',2)
% legend('GPS only','GPS + IMU')
% xlabel('x-position'); ylabel('y-position'); zlabel('z-position')







function pos_next = update_posIMU(pos_curr, vel)
% takes in a current position and velocity
deltat = 1; % seconds
pos_next = pos_curr + vel.*deltat;
end

function vdata = accel2vel(aXYZ)
% numerical interpolation of acceleration data, assuming an offset of v0=0
vX = [];
delta_t = 0.1; % seconds
for i = 1:length(aXYZ)
    vX(i) = delta_t*aXYZ(i);
end
vdata = vX;

end
