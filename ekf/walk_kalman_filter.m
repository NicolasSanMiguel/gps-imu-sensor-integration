clc
clear
close all

% % AA 272 Project Code - Winter 2022
% Custom filter


% IMU as process model, 
load('walking_data.mat');

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



load('walkingamateurfilter.mat')


figure
plot(GPSx, GPSy)
hold on; grid on;
plot(outarr(1,:), outarr(2,:),'LineWidth',2)
plot(mu(1,:), mu(3,:),'LineWidth',2)
plot(pos_i(1), pos_i(2),'gx','LineWidth',2)
legend('GPS only','Weighted Average (w_{IMU} = 0.75)',...
    'KF','Starting Point')

figure
plot3(GPSx, GPSy, GPSz)
hold on; grid on;
plot3(outarr(1,:), outarr(2,:),outarr(3,:),'LineWidth',2)
plot3(mu(1,:), mu(3,:), mu(5,:),'LineWidth',2)
plot3(pos_i(1), pos_i(2),pos_i(3),'gx','LineWidth',2)
legend('GPS only','Weighted Average (w_{IMU} = 0.75)',...
    'KF','Starting Point')

%% Export only Essential Data into CSV
data_req.complementaryfilter = outarr';
data_req.GPSx = GPSx;
data_req.GPSy = GPSy;
data_req.GPSz = GPSz;
data_req.mu = mu';
data_req.IMUonly = imu_only_positions';
struct2csv(data_req, 'walking_manzanita_3plots.csv')
