clc
clear

% logTime (s) bottomVelX (m/s), bottomVelY (m/s), bottomVelZ (m/s), rangeBottom1, rangeBottom2, rangeBottom3, heading, pitch, roll
dvlData = readmatrix('datasetGenerated/dvlData.csv');
numberOfInterpolationPointsDVL = 5;

xVelInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);
yVelInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);
zVelInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);

range1Interpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);
range2Interpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);
range3Interpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);

rollInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);
pitchInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);
yawInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);

timeInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),1);
dvlDataInterpolated = zeros((size(dvlData,1)-1)*(numberOfInterpolationPointsDVL-1),10);
indexDVL=1;
for i = 1:(size(dvlData,1)-1)
    %calculate interpolation of xyz vel and rpy
    rollInterpolatedTMP = linspace(dvlData(i,10),dvlData(i+1,10),numberOfInterpolationPointsDVL);
    pitchInterpolatedTMP = linspace(dvlData(i,9),dvlData(i+1,9),numberOfInterpolationPointsDVL);
    yawInterpolatedTMP = linspace(dvlData(i,8),dvlData(i+1,8),numberOfInterpolationPointsDVL);

    xVelInterpolatedTMP = linspace(dvlData(i,2),dvlData(i+1,2),numberOfInterpolationPointsDVL);
    yVelInterpolatedTMP = linspace(dvlData(i,3),dvlData(i+1,3),numberOfInterpolationPointsDVL);
    zVelInterpolatedTMP = linspace(dvlData(i,4),dvlData(i+1,4),numberOfInterpolationPointsDVL);

    range1InterpolatedTMP = linspace(dvlData(i,5),dvlData(i+1,5),numberOfInterpolationPointsDVL);
    range2InterpolatedTMP = linspace(dvlData(i,6),dvlData(i+1,6),numberOfInterpolationPointsDVL);
    range3InterpolatedTMP = linspace(dvlData(i,7),dvlData(i+1,7),numberOfInterpolationPointsDVL);

    timeInterpolatedTMP = linspace(dvlData(i,1),dvlData(i+1,1),numberOfInterpolationPointsDVL);
    
    
    %integrate xyz vel dependent on rpy

    for j = 2:numberOfInterpolationPointsDVL
        dvlDataInterpolated(indexDVL,10) =rollInterpolatedTMP (j);
        dvlDataInterpolated(indexDVL,9) =pitchInterpolatedTMP (j);
        dvlDataInterpolated(indexDVL,8) =yawInterpolatedTMP  (j);

        dvlDataInterpolated(indexDVL,2) =xVelInterpolatedTMP (j);
        dvlDataInterpolated(indexDVL,3) =yVelInterpolatedTMP (j);
        dvlDataInterpolated(indexDVL,4) =zVelInterpolatedTMP (j);

        dvlDataInterpolated(indexDVL,5) =range1InterpolatedTMP(j);
        dvlDataInterpolated(indexDVL,6) =range2InterpolatedTMP(j);
        dvlDataInterpolated(indexDVL,7) = range3InterpolatedTMP(j);

        dvlDataInterpolated(indexDVL,1) =timeInterpolatedTMP (j);
        indexDVL=indexDVL+1;
    end

end


writematrix(dvlDataInterpolated,"datasetGenerated/dvlDataInterpolated.csv")
%plot(yVel,xVel,".")
%axis equal

%% time,roll (deg),pitch (deg),yaw (deg),rollVel (rad/s),pitchVel (rad/s),yawVel (rad/s),accelX (m/s2),accelY (m/s2),accelZ (m/s2)

numberOfInterpolationPointsIMU = 20;
imuData = readmatrix('datasetGenerated/IMUData.csv');


rollInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);
pitchInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);
yawInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);

rollVelInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);
pitchVelInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);
yawVelInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);

accelXInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);
accelYInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);
accelZInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);

timeInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),1);

imuDataInterpolated = zeros((size(imuData,1)-1)*(numberOfInterpolationPointsIMU-1),10);
indexIMU=1;
for i = 1:(size(imuData,1)-1)
    %calculate interpolation of xyz vel and rpy
    rollInterpolatedTMP = linspace(imuData(i,10),imuData(i+1,10),numberOfInterpolationPointsIMU);
    pitchInterpolatedTMP = linspace(imuData(i,9),imuData(i+1,9),numberOfInterpolationPointsIMU);
    yawInterpolatedTMP = linspace(imuData(i,8),imuData(i+1,8),numberOfInterpolationPointsIMU);

    rollVelInterpolatedTMP = linspace(imuData(i,2),imuData(i+1,2),numberOfInterpolationPointsIMU);
    pitchVelInterpolatedTMP = linspace(imuData(i,3),imuData(i+1,3),numberOfInterpolationPointsIMU);
    yawVelInterpolatedTMP = linspace(imuData(i,4),imuData(i+1,4),numberOfInterpolationPointsIMU);

    accelXInterpolatedTMP = linspace(imuData(i,5),imuData(i+1,5),numberOfInterpolationPointsIMU);
    accelYInterpolatedTMP = linspace(imuData(i,6),imuData(i+1,6),numberOfInterpolationPointsIMU);
    accelZInterpolatedTMP = linspace(imuData(i,7),imuData(i+1,7),numberOfInterpolationPointsIMU);

    timeInterpolatedTMP = linspace(imuData(i,1),imuData(i+1,1),numberOfInterpolationPointsIMU);
    


    for j = 2:numberOfInterpolationPointsIMU
        imuDataInterpolated(indexDVL,10) = rollInterpolatedTMP (j);
        imuDataInterpolated(indexDVL,9) = pitchInterpolatedTMP (j);
        imuDataInterpolated(indexDVL,8) = yawInterpolatedTMP  (j);

        imuDataInterpolated(indexDVL,2) = rollVelInterpolatedTMP (j);
        imuDataInterpolated(indexDVL,3) = pitchVelInterpolatedTMP (j);
        imuDataInterpolated(indexDVL,4) = yawVelInterpolatedTMP (j);

        imuDataInterpolated(indexDVL,5) = accelXInterpolatedTMP(j);
        imuDataInterpolated(indexDVL,6) = accelYInterpolatedTMP(j);
        imuDataInterpolated(indexDVL,7) = accelZInterpolatedTMP(j);

        imuDataInterpolated(indexDVL,1) = timeInterpolatedTMP (j);
        indexDVL = indexDVL + 1;
    end

end


writematrix(imuDataInterpolated,"datasetGenerated/IMUDataInterpolated.csv")
%plot(yVel,xVel,".")
%axis equal


%% 'time', 'latitude', 'longitude','status','altitude','geoide altitude','true course (deg)', 'magnetic course (deg)', 'vel (knots)', 'vel (km/h)'

numberOfInterpolationPointsGT = 5;
gtData = readmatrix('datasetGenerated/GTData.csv');


latitudeInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);
longitudeInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);
statusInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);

altitudeInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);
geoideAltitudeInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);
trueCourseInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);

magneticCourseInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);
velKnotsInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);
velKmhInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);

timeInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),1);

gtDataInterpolated = zeros((size(gtData,1)-1)*(numberOfInterpolationPointsGT-1),10);
indexGT=1;
for i = 1:(size(gtData,1)-1)
    %calculate interpolation of xyz vel and rpy



    timeInterpolated = linspace(gtData(i,1),gtData(i+1,1),numberOfInterpolationPointsGT);
    latitudeInterpolated = linspace(gtData(i,2),gtData(i+1,2),numberOfInterpolationPointsGT);
    longitudeInterpolated = linspace(gtData(i,3),gtData(i+1,3),numberOfInterpolationPointsGT);
    statusInterpolated = linspace(gtData(i,4),gtData(i+1,4),numberOfInterpolationPointsGT);
    altitudeInterpolated = linspace(gtData(i,5),gtData(i+1,5),numberOfInterpolationPointsGT);
    geoideAltitudeInterpolated = linspace(gtData(i,6),gtData(i+1,6),numberOfInterpolationPointsGT);
    trueCourseInterpolated = linspace(gtData(i,7),gtData(i+1,7),numberOfInterpolationPointsGT);
    magneticCourseInterpolated = linspace(gtData(i,8),gtData(i+1,8),numberOfInterpolationPointsGT);
    velKnotsInterpolated = linspace(gtData(i,9),gtData(i+1,9),numberOfInterpolationPointsGT);
    velKmhInterpolated = linspace(gtData(i,10),gtData(i+1,10),numberOfInterpolationPointsGT);

    for j = 2:numberOfInterpolationPointsGT
                


        gtDataInterpolated(indexGT,1) = timeInterpolated(j);
        gtDataInterpolated(indexGT,2) = latitudeInterpolated (j);
        gtDataInterpolated(indexGT,3) = longitudeInterpolated(j);
        gtDataInterpolated(indexGT,4) = statusInterpolated(j);
        gtDataInterpolated(indexGT,5) = altitudeInterpolated(j);
        gtDataInterpolated(indexGT,6) = geoideAltitudeInterpolated(j);
        gtDataInterpolated(indexGT,7) = trueCourseInterpolated(j);
        gtDataInterpolated(indexGT,8) = magneticCourseInterpolated(j);
        gtDataInterpolated(indexGT,9) = velKnotsInterpolated(j);
        gtDataInterpolated(indexGT,10) = velKmhInterpolated(j);
        indexGT = indexGT + 1;
    end

end


writematrix(gtDataInterpolated,"datasetGenerated/GTDataInterpolated.csv")
%plot(yVel,xVel,".")
%axis equal