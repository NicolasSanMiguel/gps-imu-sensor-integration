clc
clear
close all

% % AA 272 Project Code - Winter 2022
% Custom filter


load('walking_data.mat');
vZ = vZ - 1;

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
    n = 1; % number of seconds ahead/behind to avg for direction vec
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
w_IMUarr = [0.75];
curr_unit = init_unit;
for j = 1:length(w_IMUarr)
    w_IMU = w_IMUarr(j);
    w_GPS = 1 - w_IMU; % relative weight of GPS data (w_IMU + w_GPS = 1)
for i = 1:cuttime % estimate subsequent positions based on GPS+IMU
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

%% Plotting
% 2D plot with different weights
figure
plot(GPSx, GPSy)
hold on; grid on;
plot(imu_only_positions(1,:), imu_only_positions(2,:),'LineWidth',2)

% plot(imu_holder(1,:), imu_holder(2,:),'LineWidth',1)
plot(outarr(1,:), outarr(2,:),'LineWidth',2)

% plot(outarr(1,:,1), outarr(2,:,1),'LineWidth',2)
% plot(outarr(1,:,2), outarr(2,:,2),'LineWidth',2)
% plot(outarr(1,:,3), outarr(2,:,3),'LineWidth',2)
% plot(outarr(1,:,4), outarr(2,:,4),'LineWidth',2)
% plot(outarr(1,:,5), outarr(2,:,5),'LineWidth',2)
% plot(outarr(1,:,6), outarr(2,:,6),'LineWidth',2)

plot(pos_i(1), pos_i(2),'gx','LineWidth',2)
legend('GPS only','IMU only','GPS + IMU, wIMU = 0.75','Starting Point','Location','best')
% legend('GPS only','IMU only','GPS + IMU, wIMU = 0',...
%     'GPS + IMU, wIMU = 0.5','GPS + IMU, wIMU = 0.75',...
%     'GPS + IMU, wIMU = 0.9','GPS + IMU, wIMU = 0.99',...
%     'GPS + IMU, wIMU = 1.0','Starting Point')
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

% 3D plot
figure
plot3(GPSx, GPSy, GPSz)
hold on; grid on;
plot3(imu_only_positions(1,:), imu_only_positions(2,:), imu_only_positions(3,:),'LineWidth',2)
plot3(outarr(1,:), outarr(2,:),outarr(3,:),'LineWidth',2)
plot3(pos_i(1), pos_i(2), pos_i(3),'gx','LineWidth',2)
legend('GPS only','IMU only','GPS + IMU','Starting Point')
xlabel('x-position'); ylabel('y-position'); zlabel('z-position')



% function pos_next = update_posIMUold(pos_curr, vel)
% % takes in a current position and velocity
% deltat = 0.1; % seconds
% pos_next = pos_curr + vel.*deltat;
% end



function pos_nextIMU = update_posIMU(pos_curr, unitvec, vel)
% takes in a current position and velocity
deltat = 10; % seconds
update_magnitudes = vel.*deltat;
pos_nextIMU = pos_curr + update_magnitudes.*unitvec;
end

% function unitdir = get_direction_vector(pos_curr, pos_next)
% vec_between = pos_next - pos_curr;
% unitdir = vec_between./norm(vec_between);
% end
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

