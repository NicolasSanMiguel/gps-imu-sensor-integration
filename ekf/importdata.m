clc
clear
close all

% AA 272 Project Code - Winter 2022

% Read IMU data
IMU_1 = readtable('run1.csv');
IMU_2 = readtable('run2.csv');
IMU_3 = readtable('run3.csv');
IMU_4 = readtable('run4.csv');
IMU_5 = readtable('run5.csv');
IMU_6 = readtable('run6.csv');
IMU_7 = readtable('run7.csv');
IMU_8 = readtable('run8.csv');

% Read GPS data
GPS_1 = readtable('SF_run_1_GPS.csv');
GPS_2 = readtable('SF_run_2_GPS.csv');
GPS_3 = readtable('SF_run_3_GPS.csv');
GPS_4 = readtable('SF_run_4_GPS.csv');
GPS_5 = readtable('SF_run_5_GPS.csv');
GPS_6 = readtable('SF_run_6_GPS.csv');
GPS_7 = readtable('SF_run_7_GPS.csv');
GPS_8 = readtable('SF_run_8_GPS.csv');

% Read gpx files from Stava
Stava_1 = gpxread('Run_1.gpx');
Stava_2 = gpxread('Run_2.gpx');
Stava_3 = gpxread('Run_3.gpx');
Stava_4 = gpxread('Run_4.gpx');
Stava_5 = gpxread('Run_5.gpx');
Stava_6 = gpxread('Run_6.gpx');
Stava_7 = gpxread('Run_7.gpx');
Stava_8 = gpxread('Run_8.gpx');


% % stationary data averaged std dev in [meters]
% xGPSavg = 8.4279
% yGPSavg = 7.3047
% zGPSavg = 5.9888


% numerical integration of IMU accelerometer data
% v = v0 + at
v0 = 0;
vX = [];
for i = 1:length(IMU_1.aX)
    vX(i) = v0 + IMU_1.aX(i);
    v0 = vX(i);
end

% figure
% plot(vX)
% hold on
% plot(IMU_1.aX)
% legend('vX','aX')


% To use: 3, 5, 8























