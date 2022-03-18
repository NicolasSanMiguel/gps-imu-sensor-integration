

clc
clear
close all


load('threeplots2error.mat');


truegpx = gpxread('true_route.gpx');
trueLat = truegpx.Latitude; %y
trueLong = truegpx.Longitude; %x

N=100;
newPoints = [];
for i = 1:length(trueLat)-1
    toadd = [linspace(trueLat(i),trueLat(i+1),N);...
             linspace(trueLong(i),trueLong(i+1),N)];
    newPoints = [newPoints toadd];
end

err2plot = [];
for w = 1:length(GPSx)
% for each GPS position, calculate the distance to the nearest point on the
% true path
curr_distances = [];
currGPS = [GPS_walk.lat GPS_walk.long];
currGPS = currGPS(w,:)';
for j = 1:length(newPoints)
    difference = newPoints(:,j) - currGPS;
%     difference(1,:)  = difference(1,:)*111.32;
%     difference(2,:)  = 5000 * cosd( difference(2,:) ) / 360;
    curr_distances(j) = norm( difference );
end
error = min(curr_distances);
err2plot(w) = error;
end




% complementary filter error
err2plotCOMP = [];
for w = 1:length(outarr(1,:))
% for each GPS position, calculate the distance to the nearest point on the
% true path
curr_distances = [];
lla = ecef2lla([outarr(1,w), outarr(2,w), outarr(3,w)]);
latty = lla(1);
longy = lla(2);
alty = lla(3);

currPOS = [latty; longy];
for j = 1:length(newPoints)
    difference = newPoints(:,j) - currPOS;
%     difference(1,:)  = difference(1,:)*111.32;
%     difference(2,:)  = 5000 * cosd( difference(2,:) ) / 360;
    curr_distances(j) = norm( difference );
end
error = min(curr_distances);
err2plotCOMP(w) = error;
end


% kalman filter error
err2plotKF = [];
for w = 1:length(outarr(1,:))
% for each GPS position, calculate the distance to the nearest point on the
% true path
curr_distances = [];
lla = ecef2lla([mu(1,w), mu(3,w), mu(5,w)]);
latty = lla(1);
longy = lla(2);
alty = lla(3);
currPOS = [latty; longy];
for j = 1:length(newPoints)
    difference = newPoints(:,j) - currPOS;
%     difference(1,:)  = difference(1,:)*111.32;
%     difference(2,:)  = 5000 * cosd( difference(2,:) ) / 360;
    curr_distances(j) = norm( difference );
end
error = min(curr_distances);
err2plotKF(w) = error;
end








figure
plot(err2plot*10^4,'LineWidth',2) % raw gps error
grid on; hold on
xlabel('Time [seconds]'); ylabel('Error [meters]')

% complementary filter error 
plot(err2plotCOMP*10^4,'LineWidth',2) % raw gps error

% kalman filter error 
plot(err2plotKF*10^4,'LineWidth',2) % raw gps error
xlim([0 205])
legend('Raw GPS','Complementary Filter','Kalman Filter','Location','best')
title('Euclidean distance from true path for Manzanita Field Test')


%% raw GPS
rawGPSerr = err2plot*10^4;
n = length(err2plot);
RMSE_rawGPS = sqrt(  (1/n)*sum( rawGPSerr.^2 ) )

% complementary
comperr = err2plotCOMP*10^4;
n = length(err2plotCOMP);
RMSE_comp = sqrt(  (1/n)*sum( comperr.^2 ) )

% kalman
kferr = err2plotKF*10^4;
n = length(err2plotKF);
RMSE_kf = sqrt(  (1/n)*sum( kferr.^2 ) )




