clc
clear
close all

% % AA 272 Project Code - Winter 2022
% Kalman filter

% IMU as process model


%% Load data
load('all_our_data.mat');

% introduce the GPS data
% GPSx = GPS_7.x;
% GPSy = GPS_7.y;
% GPSz = GPS_7.z;
XYZ = lla2ecef([GPS_4.lat, GPS_4.long, GPS_4.alt]);
GPSx = XYZ(:, 1);
GPSy = XYZ(:, 2);
GPSz = XYZ(:, 3);

% introduce the IMU data
aX = IMU_4.aX;
aY = IMU_4.aY;
aZ = IMU_4.aZ;

%% Handle IMU/GPS length mismatch
dtIMU = 0.1; % sec
dtGPS = 1; % sec

timeIMU = 0:dtIMU:dtIMU*(length(aX)-1);
timeGPS = 0:dtGPS:dtGPS*(length(GPSx)-1);
% cut the longer off at the shorter
% timeIMU(end) = 76.5000
% timeGPS(end) = 80
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

%% Clean the IMU data - normalize with 0 mean, integrate to velocity
[vX, vY, vZ] = cleanIMUdata(aX,aY,aZ);

%% KALMAN FILTER
% initial GPS position
pos_i = [GPSx(1); GPSy(1); GPSz(1)];

% Noise matrices
dt = 0.1;
% Q = [(dt^4)/4, (dt^3)/2, 0, 0;...
%     (dt^3)/2, dt^2, 0, 0;...
%     0, 0, (dt^4)/4, (dt^3)/2;...
%     0, 0, (dt^3)/2, dt^2]*0.002;

% GPS uncertainties - from GPS stationary data
varX_GPS = 8.4279^2;
varY_GPS = 7.3047^2;
varZ_GPS = 5.9888^2;
% IMU uncertainties - from IMU stationary data
varX_IMU = 0.5841^2;
varY_IMU = 0.4789^2;
varZ_IMU = 0.6951^2;

Q = 1.*eye(6);
R = diag([varX_GPS, varX_IMU, varY_GPS, varY_IMU, varZ_GPS, varZ_IMU]);
% State transition matrix and measurement matrix
F = [1, dt, 0, 0, 0, 0;...
    0, 1, 0, 0, 0, 0;...
    0, 0, 1, dt, 0, 0;...
    0, 0, 0, 1, 0, 0;...
    0, 0, 0, 0, 1, dt;...
    0, 0, 0, 0, 0, 1];
H = eye(6);

% Initial conditions
mu = [pos_i(1); 0; pos_i(2); 0; pos_i(3); 0];
P = 0.1*eye(6);

% Run Kalman Filter
for i = 2:length(timeGPS)
    % Predict
    mu(:,i) = F*mu(:,i-1);
    P(:,:,i) = F*P(:,:,i-1)*F' + Q;

    % Update
    z(:,i) = [GPSx(i); vX(i); GPSy(i); vY(i); GPSz(i); vZ(i)];
    y_prefit(:,i) = z(:,i) - H*mu(:,i);
    S = R + H*P(:,:,i)*H';
    K = P(:,:,i)*H'*inv(S);
    mu(:,i) = mu(:,i) + K*y_prefit(:,i);
    P(:,:,i) = (eye(6) - K*H)*P(:,:,i)*(eye(6) - K*H)' + K*R*K';
    y_postfit(:,i) = z(:,i) - H*mu(:,i);
end



% load('walkingamateurfilter.mat')
outarr = run_complementary_filter(vX,vY,vZ,GPSx,GPSy,GPSz,cuttime);

figure
plot(GPSx, GPSy)
hold on; grid on;
plot(outarr(1,:), outarr(2,:),'LineWidth',2) % supposed to be the
% complementary filter
plot(mu(1,:), mu(3,:),'LineWidth',2)
plot(pos_i(1), pos_i(2),'gx','LineWidth',2)
legend('GPS only','Complementary Filter, wIMU = 0.97',...
    'KF','Starting Point','Location','best')


%% Export only Essential Data into CSV
data_req.complementaryfilter = outarr';
data_req.GPSx = GPSx;
data_req.GPSy = GPSy;
data_req.GPSz = GPSz;
data_req.mu = mu';
% data_req.IMUonly = imu_only_positions';
struct2csv(data_req, 'run4_filtered.csv')






























function [vX, vY, vZ] = cleanIMUdata(aX,aY,aZ)


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
vZ = vZ-1;

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




function outarr = run_complementary_filter(vX,vY,vZ,GPSx,GPSy,GPSz,cuttime);

% initial GPS position
pos_i = [GPSx(1); GPSy(1); GPSz(1)];

% % % % % % % % % IMU only track
% this loop is separate because pos_curr must be different, and i didn't
% want to have two different pos_curr's in one loop
pos_curr = pos_i;
imu_only_positions = [];

% init direct. is 2ndpos - 1stpos
% initial_direction = [mean(GPSx(10:20)); mean(GPSy(10:20)); mean(GPSz(10:20))]...
%     - [mean(GPSx(1:10)); mean(GPSy(1:10)); mean(GPSz(1:10))];
initial_direction = [mean(GPSx(1:50)); mean(GPSy(1:50)); mean(GPSz(1:50))]...
    - pos_i;
init_unit = initial_direction ./ norm(initial_direction);
curr_unit = init_unit;
for i = 1:cuttime*10 % estimate subsequent positions based on GPS+IMU
    nextIMU = update_posIMU(pos_curr, curr_unit, [vX(i); vY(i); vZ(i)]); % next point only relying on IMU
    %         nextIMU = update_posIMUold(pos_curr, [mean(vX(1+(i-1)*10:1+10*i)); ...
    %                                        mean(vY(1+(i-1)*10:1+10*i)); ...
    %                                        mean(vZ(1+(i-1)*10:1+10*i))]);
    currGPStime = floor(i/10); % sec
    p = currGPStime; % just to be less verbose
    n = 10; % number of seconds ahead/behind to avg for direction vec
    try
        curr_direction = [mean(GPSx(p+n:p+2*n)); mean(GPSy(p+n:p+2*n)); mean(GPSz(p+n:p+2*n))]...
            - [mean(GPSx(p+1:p+n)); mean(GPSy(p+1:p+n)); mean(GPSz(p+1:p+n))];
        curr_unit = curr_direction ./ norm(curr_direction);
    catch
        curr_unit = curr_unit;
    end


    if mod(i,100)==0 % to print curr_unit to make sure it's changing direction
        curr_unit = curr_unit;
    end


    imu_only_positions(:,i) = nextIMU;
    pos_curr = nextIMU;
end

% % % % % % % % % IMU + GPS Filter
pos_curr = pos_i;
filtered_positions = [];
outarr = [];
imu_holder = [];
w_IMUarr = [0.97];
curr_unit = init_unit;
for j = 1:length(w_IMUarr)
    w_IMU = w_IMUarr(j);
    w_GPS = 1 - w_IMU; % relative weight of GPS data (w_IMU + w_GPS = 1)
    for i = 1:cuttime-1 % estimate subsequent positions based on GPS+IMU
        nextGPS = [GPSx(i+1); GPSy(i+1); GPSz(i+1)]; % next GPS measurement

        % next point only relying on IMU, bins 10 IMU measurements taken in 1
        % second (GPS sample rate is 1 second)
        curr_unit = give_me_curr_unit(i, curr_unit, GPSx,GPSy,GPSz);

        nextIMU = update_posIMU(pos_curr,  curr_unit, ...
            [mean(vX(1+(i-1)*10:1+10*i)); ...
            mean(vY(1+(i-1)*10:1+10*i)); ...
            mean(vZ(1+(i-1)*10:1+10*i))]);
        % weighted average of GPS and IMU next points
        midpoint = [(w_GPS*nextGPS(1) + w_IMU*nextIMU(1));
            (w_GPS*nextGPS(2) + w_IMU*nextIMU(2));
            (w_GPS*nextGPS(3) + w_IMU*nextIMU(3))];

        filtered_positions(:,i) = midpoint;
        pos_curr = midpoint;
    end
    outarr(:,:,j) = filtered_positions(:,:);
end

% %% Plotting
% % 2D plot with different weights
% figure
% plot(GPSx, GPSy)
% hold on; grid on;
% plot(imu_only_positions(1,:), imu_only_positions(2,:),'LineWidth',2)
% 
% % plot(imu_holder(1,:), imu_holder(2,:),'LineWidth',1)
% plot(outarr(1,:), outarr(2,:),'LineWidth',2)
% 
% % plot(outarr(1,:,1), outarr(2,:,1),'LineWidth',2)
% % plot(outarr(1,:,2), outarr(2,:,2),'LineWidth',2)
% % plot(outarr(1,:,3), outarr(2,:,3),'LineWidth',2)
% % plot(outarr(1,:,4), outarr(2,:,4),'LineWidth',2)
% % plot(outarr(1,:,5), outarr(2,:,5),'LineWidth',2)
% % plot(outarr(1,:,6), outarr(2,:,6),'LineWidth',2)
% 
% plot(pos_i(1), pos_i(2),'gx','LineWidth',2)
% legend('GPS only','IMU only','GPS + IMU, wIMU = 0.75','Starting Point','Location','best')
% % legend('GPS only','IMU only','GPS + IMU, wIMU = 0',...
% %     'GPS + IMU, wIMU = 0.5','GPS + IMU, wIMU = 0.75',...
% %     'GPS + IMU, wIMU = 0.9','GPS + IMU, wIMU = 0.99',...
% %     'GPS + IMU, wIMU = 1.0','Starting Point')
% xlabel('x-position'); ylabel('y-position')


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
% plot3(imu_only_positions(1,:), imu_only_positions(2,:), imu_only_positions(3,:),'LineWidth',2)
% plot3(outarr(1,:), outarr(2,:),outarr(3,:),'LineWidth',2)
% plot3(pos_i(1), pos_i(2), pos_i(3),'gx','LineWidth',2)
% legend('GPS only','IMU only','GPS + IMU','Starting Point')
% xlabel('x-position'); ylabel('y-position'); zlabel('z-position')
end


function pos_nextIMU = update_posIMU(pos_curr, unitvec, vel)
% takes in a current position and velocity
deltat = 10; % seconds
update_magnitudes = vel.*deltat;
pos_nextIMU = pos_curr + update_magnitudes.*unitvec;
end


function curr_unit = give_me_curr_unit(i, curr_unit, GPSx,GPSy,GPSz)
p = i;
n = 7; % number of seconds ahead/behind to avg for direction vec
try
    curr_direction = [mean(GPSx(p+n:p+2*n)); mean(GPSy(p+n:p+2*n)); mean(GPSz(p+n:p+2*n))]...
        - [mean(GPSx(p+1:p+n)); mean(GPSy(p+1:p+n)); mean(GPSz(p+1:p+n))];
    curr_unit = curr_direction ./ norm(curr_direction);
catch
    curr_unit = curr_unit;
end


end






