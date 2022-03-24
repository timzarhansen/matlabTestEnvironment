clc
clear

% logTime (s) bottomVelX (m/s), bottomVelY (m/s), bottomVelZ (m/s), rangeBottom1, rangeBottom2, rangeBottom3, heading, pitch, roll
dvlData = readmatrix('datasetGenerated/dvlData.csv');
numberOfInterpolationPoints = 10;
xPosition = zeros((size(dvlData,1)-1)*(numberOfInterpolationPoints-1),1);
yPosition = zeros((size(dvlData,1)-1)*(numberOfInterpolationPoints-1),1);
zPosition = zeros((size(dvlData,1)-1)*(numberOfInterpolationPoints-1),1);
overallTransformation = eye(4);
indexPlotPositions=1;
for i = 1:(size(dvlData,1)-1)
    %calculate interpolation of xyz vel and rpy
    rollInterpolated = linspace(dvlData(i,10),dvlData(i+1,10),numberOfInterpolationPoints);
    pitchInterpolated = linspace(dvlData(i,9),dvlData(i+1,9),numberOfInterpolationPoints);
    yawInterpolated = linspace(dvlData(i,8),dvlData(i+1,8),numberOfInterpolationPoints);
    xVelInterpolated = linspace(dvlData(i,2),dvlData(i+1,2),numberOfInterpolationPoints);
    yVelInterpolated = linspace(dvlData(i,3),dvlData(i+1,3),numberOfInterpolationPoints);
    zVelInterpolated = linspace(dvlData(i,4),dvlData(i+1,4),numberOfInterpolationPoints);
    timeInterpolated = linspace(dvlData(i,1),dvlData(i+1,1),numberOfInterpolationPoints);
    %integrate xyz vel dependent on rpy

    for j = 1:numberOfInterpolationPoints-1
        %calculate transformation matrix
        tmpTrans = eye(4);
        tmpTrans(1:3,1:3) = rotationMatrix(rollInterpolated(j),pitchInterpolated(j),yawInterpolated(j));
        tmpTrans(1,4) = xVelInterpolated(j)*(timeInterpolated(j+1)-timeInterpolated(j));
        tmpTrans(2,4) = yVelInterpolated(j)*(timeInterpolated(j+1)-timeInterpolated(j));
        tmpTrans(3,4) = zVelInterpolated(j)*(timeInterpolated(j+1)-timeInterpolated(j));
        overallTransformation=overallTransformation*tmpTrans;

        xPosition(indexPlotPositions)=overallTransformation(1,4);
        yPosition(indexPlotPositions)=overallTransformation(2,4);
        zPosition(indexPlotPositions)=overallTransformation(3,4);
        indexPlotPositions=indexPlotPositions+1;
    end

end

plot(yPosition,xPosition,".")
axis equal