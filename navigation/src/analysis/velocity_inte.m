clc;
close all;

%open bag file
bag = rosbag('../data/boston_tour.bag');


imu = select(bag,'Topic','/imu');
msgStr1 = readMessages(imu,'DataFormat','struct');
gps = select(bag,'Topic','/gps');
msgStr2 = readMessages(gps,'DataFormat','struct');

magX = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStr1);
magY = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStr1);
magZ = cellfun(@(m) double(m.MagField.MagneticField_.Z),msgStr1);

omegaX = cellfun(@(m) double(m.Imu.AngularVelocity.X),msgStr1);
omegaY = cellfun(@(m) double(m.Imu.AngularVelocity.Y),msgStr1);
omegaZ = cellfun(@(m) double(m.Imu.AngularVelocity.Z),msgStr1);

accX = cellfun(@(m) double(m.Imu.LinearAcceleration.X),msgStr1);
accY = cellfun(@(m) double(m.Imu.LinearAcceleration.Y),msgStr1);
accZ = cellfun(@(m) double(m.Imu.LinearAcceleration.Z),msgStr1);

orientationX = cellfun(@(m) double(m.Imu.Orientation.X),msgStr1);
orientationY = cellfun(@(m) double(m.Imu.Orientation.Y),msgStr1);
orientationZ = cellfun(@(m) double(m.Imu.Orientation.Z),msgStr1);
orientationW = cellfun(@(m) double(m.Imu.Orientation.W),msgStr1);

utmEasting = cellfun(@(m) double(m.UTMEasting),msgStr2);
utmNorthing = cellfun(@(m) double(m.UTMNorthing),msgStr2);

gpsSec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStr2);
gpsNanoSec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStr2);
gpsTimeTot = double(gpsSec + ( gpsNanoSec * 10^(-9)));
gpsTime = gpsTimeTot - gpsTimeTot(1);

imuSec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStr1);
imuNanoSec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStr1);
imuTimeTot = double(imuSec + ( imuNanoSec * 10^(-9)));
imuTime = imuTimeTot - imuTimeTot(1);

%quat to euler
quat = [orientationW orientationX orientationY orientationZ];
zyxRadEul = quat2eul(quat);
yaw = zyxRadEul (:,1);
pitch = zyxRadEul (:,2);
roll = zyxRadEul (:,3);

%integrate acceleration to velocity
imuVel = cumtrapz(imuTime,accX);
gpsVel = zeros(length(utmEasting),1);
utmDistance = zeros(length(utmEasting),1);
gpsTime_d =  zeros(length(utmEasting),1);
for i = 2 : length(utmEasting)

    utmDistance(i-1) = sqrt((utmEasting(i)-utmEasting(i-1))^2+(utmNorthing(i)-utmNorthing(i-1))^2);
    gpsVel(1) =  sqrt((utmEasting(2)-utmEasting(1))^2+(utmNorthing(2)-utmNorthing(1))^2);   
    gpsTime_d(i-1) = abs(gpsTime(i) - gpsTime(i-1));
    if gpsTime_d(i-1) ==0
        gpsVel(i) = gpsVel(i-1);
    else
        gpsVel(i) = (utmDistance(i-1)/gpsTime_d(i-1));  
    end
end
gpsTime = sort(gpsTime);

figure;
plot(imuTime, imuVel, "DisplayName","Forward velocity from accelerometer",'LineWidth',2.0);
hold on;
plot(gpsTime, gpsVel, "DisplayName","Forward velocity from GPS ",'LineWidth',2.0);
xlabel('time (s)')
ylabel('vel (m/s)')
title('Forward velocity from accelerometer vs velocity from GPS')
legend;

% Velocity estimate from gps
figure;
plot(gpsTime, gpsVel, "DisplayName","Forward velocity from GPS ",'LineWidth',2.0);
xlabel('time (s)')
ylabel('vel (m/s)')
title('Velocity estimate from gps')

% moving average filter - accelerometer

a = 0.992;
accelFilt = zeros(length(accX),1);
accelFilt(1) = accX(1);
for i = 2 : length(accX)    
    accelFilt(i) = (1-a)*accX(i) + a * accelFilt(i-1);
end

%dynamically change our acceleration values based on gps velocity
accelChng = diff(accelFilt);
len = 0;
startEndTime = [];
for i = 2 : length(accelChng)
    if abs(accelChng(i)) < 0.001
        len = len + 1;
    else
        if len > 1*40
            startC = int16(imuTime(i-1-len));
            endC = int16(imuTime(i-1));

            a = find(gpsTime>startC,1,'first');
            b = find(gpsTime>endC,1,'first')-1;

            if gpsVel(a:b,1) < 0.7
                startEndTime = [startEndTime,i-1-len];
                startEndTime = [startEndTime,i-1];
            end
        end
        len = 0;
    end
end

%dynamic bias removal
startEnd = length(startEndTime);
for i = 2 : (startEnd)
    bias = mean(accelFilt(startEndTime(i-1):startEndTime(i)));
    if i==2
        start = 1;
    else
        start = startEndTime(i-1);
    end   
    if i < startEnd -1
        end_ = startEndTime(i);
    else
        end_ = length(accelFilt);
    end
    for j = start : end_
        accelCorrect(j) = accelFilt(j)- bias;
    end
end


correctVel = cumtrapz(imuTime,accelCorrect);
correctVel(end) = 0;
gpsVel(length(gpsVel)) =0;

figure;
plot(imuTime,accX,"DisplayName"," Accel - before adj", 'LineWidth',2.0);
hold on;
plot(imuTime,accelFilt,"DisplayName","Accel - after smoothening", 'LineWidth',2.0);
hold on;
plot(imuTime,accelCorrect,"DisplayName","Accel - after adj", 'LineWidth',2.0);
% hold on;
xlabel('time (s)')
ylabel('accel (m/s^2)')
title('Acceleration before and after adjustment')
legend


figure;
plot(gpsTime,gpsVel,"DisplayName"," Velocity estimate - GPS ", 'LineWidth',2.0);
hold on;
plot(imuTime,correctVel,"DisplayName","Vel (accel) - after adj", 'LineWidth',2.0);
xlabel('time (s)')
ylabel('vel (m/s)')
title('Adjusted velocity from IMU vs GPS velocity')
legend

figure;
plot(imuTime, imuVel, "DisplayName","Forward velocity from accelerometer",'LineWidth',2.0);
hold on;
plot(imuTime,correctVel,"DisplayName","Vel (accel) - after adj", 'LineWidth',2.0);
xlabel('time (s)')
ylabel('vel (m/s)')
title(' Forward velocity from accel (before and after adjustment)')
legend












