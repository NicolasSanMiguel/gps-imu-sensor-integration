clc
clear
close all

% % AA 272 Project Code - Winter 2022
% Extended Kalman Filter



% IMU as process model, 
load('walking_dataEKF.mat');

% initial GPS position and direction
pos_i = [GPSx(1); GPSy(1); GPSz(1)];
init_heading_vec = [GPSlat(30); GPSlong(30); GPSalt(30)] - [GPSlat(1); GPSlong(1); GPSalt(1)];
init_heading = init_heading_vec./norm(init_heading_vec);

init_beta = atan2(init_heading(1),init_heading(2));
init_beta = 360 - rad2deg(init_beta) + 90; % to align with N
init_beta = wrapTo360(init_beta);

% Noise matrices
dt = 0.1;
% Q = [(dt^4)/4, (dt^3)/2, 0, 0;...
%     (dt^3)/2, dt^2, 0, 0;...
%     0, 0, (dt^4)/4, (dt^3)/2;...
%     0, 0, (dt^3)/2, dt^2]*0.002;

% GPS uncertainties - from GPS stationary data
varX_GPS = 8.4279^2; % meters^2
varY_GPS = 7.3047^2; % meters^2
varZ_GPS = 5.9888^2; % meters^2

% IMU accelerometer uncertainties - from lookAtStationaryData.m
varX_IMU = 0.5841^2; 
varY_IMU = 0.4789^2; 
varZ_IMU = 0.6951^2; 
% variations in velocity are propostional to variations in acceleration
varVX_IMU = varX_IMU;
varVY_IMU = varX_IMU;
varVZ_IMU = varX_IMU;

% IMU gyro uncertainties - from lookAtStationaryData.m
varX_gyro = 0.032236^2;
varY_gyro = 0.030449^2;
varZ_gyro = 0.03789^2;

% variance of the biases
varbX = 0.0001;
varbY = 0.0001;
varbZ = 0.0001;
varbgX = 0.000001;
varbgY = 0.000001;
varbgZ = 0.000001;

Q = diag([varX_IMU,varY_IMU,varZ_IMU, ...
             varVX_IMU,varVY_IMU,varVZ_IMU, ...
             varX_gyro, varY_gyro, varZ_gyro, ...
             varbX, varbY, varbZ,...
             varbgX, varbgY, varbgZ]);
R = 0.1*diag([varX_GPS, varY_GPS, varZ_GPS]);

% State transition matrix
beta = init_beta;

% Measurement matrix
H = zeros([3,15]);
H(1,1) = 1;
H(2,2) = 1;
H(3,3) = 1;

% Initial conditions
mu(:,1) = [pos_i(1); pos_i(2); pos_i(3); ...
      0; 0; 0;...
      0; 0; init_beta;...
      0;0;0;0;0;0];
P = 0.1*eye(15);

mu_curr = state_trans_mdl(mu(:,1),mean(gZ(1:10)),...
                                      mean(aX(1:10)),...
                                      mean(aY(1:10)),...
                                      mean(aZ(1:10)  ));

% State transition matrix
% this is calculated in the getF function
global Aprev
Aprev = zeros([3, 3]);



% Run Extended Kalman Filter
for i = 2:length(timeGPS)-1

    % Predict
    mu(:,i) = state_trans_mdl(mu_curr,mean(gZ((i*10-9):(i*10))),...
                                      mean(aX(i*10-9):(i*10)),...
                                      mean(aY(i*10-9):(i*10)),...
                                      mean(aZ(i*10-9):(i*10))  );
    F = getF( mu(:,i) );
    P(:,:,i) = F*P(:,:,i-1)*F' + Q;
    
    % Update
    z(:,i) = [GPSx(i); GPSy(i); GPSz(i)];
    y_prefit(:,i) = z(:,i) - H*mu(:,i);
    S = R + H*P(:,:,i)*H';
    K = P(:,:,i)*H'*inv(S);
    mu(:,i) = mu(:,i) + K*y_prefit(:,i);
    P(:,:,i) = (eye(15) - K*H)*P(:,:,i)*(eye(15) - K*H)' + K*R*K';
    y_postfit(:,i) = z(:,i) - H*mu(:,i);
end



% load('walkingamateurfilter.mat')


figure
plot(GPSx, GPSy)
hold on; grid on;
plot(mu(1,:), mu(2,:),'LineWidth',2)
plot(pos_i(1), pos_i(2),'gx','LineWidth',2)
legend('GPS only','EKF','Starting Point')

figure
plot3(GPSx, GPSy, GPSz)
hold on; grid on;
plot3(mu(1,:), mu(2,:), mu(3,:),'LineWidth',2)
plot3(pos_i(1), pos_i(2),pos_i(3),'gx','LineWidth',2)
legend('GPS only','EKF','Starting Point')

% %% Export only Essential Data into CSV
% data_req.complementaryfilter = outarr';
% data_req.GPSx = GPSx;
% data_req.GPSy = GPSy;
% data_req.GPSz = GPSz;
% data_req.mu = mu';
% data_req.IMUonly = imu_only_positions';
% struct2csv(data_req, 'walking_manzanita_EKF.csv')

function  F = getF( mu )
[lat, long, ~] = ecef2lla([mu(1) mu(2) mu(3)]);
beta = mu(9);
phi = lat;
lambda = long;
global Aprev

% this should be the derivatives
R_body2ENU = [cosd(beta), sind(beta), 0; ...
             -sind(beta), cosd(beta), 0; ...
                      0,         0, 1];
R_ENU2ECEF = [-sin(lambda), -cos(lambda)*sin(phi), cos(lambda)*cos(phi); ...
             cos(lambda), -sin(lambda)*sin(phi), sin(lambda)*cos(phi); ...
                      0,         cos(phi), sin(phi)];
A = R_ENU2ECEF*R_body2ENU;
deltaA = A - Aprev;
Aprev = A;
dt = 10;
F = eye(15);
F(1,4) = dt;
F(2,5) = dt;
F(3,6) = dt;
F(4,4) = A(1,1).*dt;
F(5,4) = A(2,2).*dt;
F(6,4) = A(3,3).*dt;


end

function mu_next = state_trans_mdl(mu,gZ, aX, aY, aZ)
% include the state transition model f(.)
deltat = 1;

[lat, long, ~] = ecef2lla([mu(1) mu(2) mu(3)]);
beta = mu(9);
phi = lat;
lambda = long;

R_body2ENU = [cosd(beta), sind(beta), 0; ...
             -sind(beta), cosd(beta), 0; ...
                      0,         0, 1];
R_ENU2ECEF = [-sin(lambda), -cos(lambda)*sin(phi), cos(lambda)*cos(phi); ...
             cos(lambda), -sin(lambda)*sin(phi), sin(lambda)*cos(phi); ...
                      0,         cos(phi), sin(phi)];
A = R_ENU2ECEF*R_body2ENU;

R2 = 10 .* eye(3);
Rnoisy = sqrt(R2)*randn(1);

vX = mu(4) + Rnoisy(1,1);
vY = mu(5) + Rnoisy(2,2);
vZ = mu(6) + Rnoisy(3,3);

mu_next = mu;
mu_next(1:3) = mu(1:3) + deltat.*A*[vX; vY; vZ];
mu_next(4) = (aX+mu(10))*deltat;
mu_next(5) = (aY+mu(11))*deltat;
mu_next(6) = (aZ+mu(12))*deltat;
mu_next(9) = mu(9) + deltat*(gZ+mu(15));
end

