clc;
close all;

%open bag file
bag = rosbag('../data/boston_tour.bag');


% imu data
imu = select(bag,'Topic','/imu');
msgStr = readMessages(imu,'DataFormat','struct');

magX = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStr);
magY = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStr);
magZ = cellfun(@(m) double(m.MagField.MagneticField_.Z),msgStr);

omegaX = cellfun(@(m) double(m.Imu.AngularVelocity.X),msgStr);
omegaY = cellfun(@(m) double(m.Imu.AngularVelocity.Y),msgStr);
omegaZ = cellfun(@(m) double(m.Imu.AngularVelocity.Z),msgStr);

orientationX = cellfun(@(m) double(m.Imu.Orientation.X),msgStr);
orientationY = cellfun(@(m) double(m.Imu.Orientation.Y),msgStr);
orientationZ = cellfun(@(m) double(m.Imu.Orientation.Z),msgStr);
orientationW = cellfun(@(m) double(m.Imu.Orientation.W),msgStr);

accX = cellfun(@(m) double(m.Imu.LinearAcceleration.X),msgStr);
accY = cellfun(@(m) double(m.Imu.LinearAcceleration.Y),msgStr);
accZ = cellfun(@(m) double(m.Imu.LinearAcceleration.Z),msgStr);

imuSec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStr);
imuNanoSec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStr);
imuTimeTot = double(imuSec + ( imuNanoSec * 10^(-9)));
imuTime = imuTimeTot - imuTimeTot(1);

%quat to euler
quat = [orientationW orientationX orientationY orientationZ];
zyxEulRad = quat2eul(quat);
yaw = zyxEulRad (:,1);
pitch = zyxEulRad (:,2);
roll = zyxEulRad (:,3);

%calibration matrix - from magnetometer calibration.m
scaleMat = [0.593711065086482,0;0,1];
offsetX = -0.071906414507347;
offsetY = 0.212860721801080;
correctedX = magX - offsetX;
correctedY = magY - offsetY;
magCalibrated =  (scaleMat*[correctedX,correctedY]')';

% yaw from magnetometer
magYawCalibrated = (atan2(-magCalibrated(:,2),magCalibrated(:,1)));
magYawRaw = atan2(-magY,magX);
magYawUnwrapped = unwrap(magYawCalibrated);
figure;
plot(imuTime, magYawCalibrated, "DisplayName"," Corrected yaw",'LineWidth',2.0);
hold on;
plot(imuTime,magYawRaw,"DisplayName"," Raw magnetometer yaw",'LineWidth',2.0);
xlabel('time (s)')
ylabel('yaw (rad)')
title('Comparision of Yaw Angles - Magnetometer')
legend;

% yaw from gyroscope
gyroYaw = cumtrapz(imuTime,omegaZ)+ magYawCalibrated(1);
gyroYawWrapped = wrapToPi(gyroYaw);
figure;
plot(imuTime,magYawCalibrated,"DisplayName"," Corrected Magnetometer Yaw",'LineWidth',2.0);
hold on;
plot(imuTime,gyroYawWrapped,"DisplayName","Yaw from Gyro ",'LineWidth',2.0);
xlabel('time (s)')
ylabel('yaw (rad)')
title('Magnetometer vs Yaw from Gyro')
legend;
 
%Low pass filter on magnetometer
magLowPass= lowpass(magYawUnwrapped, 0.001, 40);

%high pass filter on gyro yaw
gyroHighPass = highpass(gyroYaw,0.01,40);

%complemetary filter
a_c = 0.4;
filteredYaw = a_c*magLowPass + (1-a_c)*gyroHighPass;

figure;
plot(imuTime,(magLowPass),"DisplayName","Yaw (Mag) - low pass filter",'LineWidth',2.0);
hold on;
plot(imuTime,(gyroHighPass),"DisplayName","Yaw (Gyro) - high pass filter",'LineWidth',2.0);
hold on;
plot(imuTime, (filteredYaw), "DisplayName","Yaw - Complementary filter",'LineWidth',2.0);
hold on;
xlabel('time (s)')
ylabel('yaw (rad)')
title('Low pass vs high pass vs complementary filters')
legend;


%complementary filter vs imu yaw
figure;
plot(imuTime,unwrap(yaw),"DisplayName","Yaw from IMU",'LineWidth',2.0);
hold on;
plot(imuTime, (filteredYaw), "DisplayName","Yaw - Complementary filter",'LineWidth',2.0);
hold on;
xlabel('time (s)')
ylabel('yaw (rad)')
title('Yaw from Complementary filter vs Yaw from IMU')
legend;