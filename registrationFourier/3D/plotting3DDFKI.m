clc
clear


fname = 'csvFiles/fullGraphDataset.csv'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
dataSet = jsondecode(str);

%% plotting 2D map First


% size(dataSet.keyFrames,1)
dimensionMap = 256;
map = zeros(dimensionMap,dimensionMap);
mapIndex = zeros(dimensionMap,dimensionMap);
sizeOfMap = 35;


for i = 1:(size(dataSet.keyFrames,1)-30000)
    
    if dataSet.keyFrames(i).intensityValues.type ~= 5

        xPos = dataSet.keyFrames(i).position.x;
        yPos = dataSet.keyFrames(i).position.y;
        zPos = dataSet.keyFrames(i).position.z;
        roll = dataSet.keyFrames(i).position.roll;
        pitch = dataSet.keyFrames(i).position.pitch;
        yaw = dataSet.keyFrames(i).position.yaw;
        poseSonar = transformationsMatrix(roll,pitch,yaw,xPos,yPos,zPos);
        localRotationIntensityRay = transformationsMatrix(0,0,dataSet.keyFrames(i).intensityValues.angle,0,0,0);
        
    
        ignoreDistanceIndex = round((0.5 / (dataSet.keyFrames(i).intensityValues.range))*(size(dataSet.keyFrames(i).intensityValues.intensity,1)));
    
    
        for j = ignoreDistanceIndex:size(dataSet.keyFrames(i).intensityValues.intensity,1)
    
            distanceOfIntensity = j / size(dataSet.keyFrames(i).intensityValues.intensity,1) *dataSet.keyFrames(i).intensityValues.range;
            % incrementOfScan = dataSet.keyFrames(i).intensityValues.intensity.range/size(dataSet.keyFrames(i).intensityValues.intensity,1);
            incrementOfScan = 1;
    
            for incrementTMP = -incrementOfScan - 5:incrementOfScan + 5
                positionOfIntensity = [distanceOfIntensity,0,0,1];
                rotationOfPoint = incrementTMP / 400.0;
                rotationForBetterView = transformationsMatrix(0,0,rotationOfPoint,0,0,0);
                positionOfIntensity = rotationForBetterView * positionOfIntensity';
                positionOfIntensity = poseSonar * localRotationIntensityRay * positionOfIntensity;
                indexX = round(positionOfIntensity(1) / (sizeOfMap / 2) * dimensionMap /2 + dimensionMap / 2)+1;
                indexY = round(positionOfIntensity(2) / (sizeOfMap / 2) * dimensionMap /2 +dimensionMap / 2)+1;
                if  indexX <= dimensionMap && indexY <= dimensionMap && indexY > 0 && indexX > 0
                    map(indexX,indexY) = map(indexX,indexY)+dataSet.keyFrames(i).intensityValues.intensity(j);
                    mapIndex(indexX,indexY) = mapIndex(indexX,indexY)+1;
                end
            end
        end
    end
end




for i = 1:dimensionMap
    for j = 1:dimensionMap
        if(mapIndex(i,j)>0)
            map(i,j) = map(i,j)/mapIndex(i,j);
        end 
    end
end


imagesc(map)




%% 3D map



dimension3DMap = 128;
map3D = zeros(dimension3DMap,dimension3DMap,dimension3DMap);
map3DIndex = zeros(dimension3DMap,dimension3DMap,dimension3DMap);
sizeOfMap = 35;

overallMean= 0;
numberOfMeanElements=0;

for i = 1:(size(dataSet.keyFrames,1)-100)

    if dataSet.keyFrames(i).intensityValues.type == 5 
        currentAngle = dataSet.keyFrames(i).intensityValues.angle;
        currentAngle = mod(currentAngle +2*pi,2*pi);
        if currentAngle>pi
            currentAngle = currentAngle-2*pi;
        end
        % if abs(abs(currentAngle)-pi/2)<0.1 || abs(abs(currentAngle)-pi)<1.5
        if abs(abs(currentAngle)-pi)<1.0




            xPos = dataSet.keyFrames(i).position.x;
            yPos = dataSet.keyFrames(i).position.y;
            zPos = dataSet.keyFrames(i).position.z;
            roll = dataSet.keyFrames(i).position.roll;
            pitch = dataSet.keyFrames(i).position.pitch;
            yaw = dataSet.keyFrames(i).position.yaw;
            poseSonar = transformationsMatrix(roll,pitch,yaw,xPos,yPos,zPos);
            angleOfSonar = dataSet.keyFrames(i).intensityValues.angle;
            angleOfSonarMatrix = transformationsMatrix(angleOfSonar+pi,0,0,0,0,0);
            posDiffSonarMatrix = transformationsMatrix(0,0,0,0.4,0,0);
    
    
            % localRotationIntensityRay = transformationsMatrix(0,0,dataSet.keyFrames(i).intensityValues.angle,0,0,0);
            % localRotationIntensityRay = transformationsMatrix(angleOfSonar+pi,0,0,0,0,0)*transformationsMatrix(0,0,0,0.4,0,0);
            
        
            ignoreDistanceIndex = round((1.0 / (dataSet.keyFrames(i).intensityValues.range))*(size(dataSet.keyFrames(i).intensityValues.intensity,1)));
        
            addedSum  = 0;
            for j = ignoreDistanceIndex:size(dataSet.keyFrames(i).intensityValues.intensity,1)
        
                distanceOfIntensity = j / size(dataSet.keyFrames(i).intensityValues.intensity,1) *dataSet.keyFrames(i).intensityValues.range;
                % incrementOfScan = dataSet.keyFrames(i).intensityValues.range/size(dataSet.keyFrames(i).intensityValues.intensity,1);
                % incrementOfScan = 4;
                addedSum   =   addedSum+dataSet.keyFrames(i).intensityValues.intensity(j);
                % 0.007853662054643;
                if addedSum <500 && distanceOfIntensity<5
                    for incrementTMP = -5:5% roll
                        for incrementotherTMP = -60:60 % Y
                            positionOfIntensity = [0,0,distanceOfIntensity,1];
                            rotationOfPoint = incrementTMP / 800.0;
                            rotationForBetterView = transformationsMatrix(rotationOfPoint,0,0,0,0,0);
                            % positionOfIntensity = rotationForBetterView *transformationsMatrix(0,incrementotherTMP/800.0,0,0,0,0)*angleOfSonarMatrix* posDiffSonarMatrix*positionOfIntensity';
                            positionOfIntensity =  posDiffSonarMatrix*transformationsMatrix(0,incrementotherTMP/720.0,0,0,0,0)*angleOfSonarMatrix*rotationForBetterView*positionOfIntensity';

                            positionOfIntensity = poseSonar * positionOfIntensity;
                            indexX = round(positionOfIntensity(1) / (sizeOfMap / 2) * dimension3DMap /2 + dimension3DMap / 2)+1;
                            indexY = round(positionOfIntensity(2) / (sizeOfMap / 2) * dimension3DMap /2 +dimension3DMap / 2)+1;
                            indexZ = round(positionOfIntensity(3) / (sizeOfMap / 2) * dimension3DMap /2 +dimension3DMap / 2)+1;
                            %&& indexZ>dimension3DMap/2+1
                            if  indexX <= dimension3DMap && indexY <= dimension3DMap && indexZ <= dimension3DMap && indexY > 0 && indexX > 0 && indexZ > 0  
                                map3D(indexX,indexY,indexZ) = map3D(indexX,indexY,indexZ)+dataSet.keyFrames(i).intensityValues.intensity(j);
                                map3DIndex(indexX,indexY,indexZ) = map3DIndex(indexX,indexY,indexZ)+1;
                            end
                        end
                    end
                end
            end
            numberOfMeanElements = numberOfMeanElements+1;
            overallMean = overallMean+addedSum;
        end
    end


    if dataSet.keyFrames(i).intensityValues.type ~= 5

        xPos = dataSet.keyFrames(i).position.x;
        yPos = dataSet.keyFrames(i).position.y;
        zPos = dataSet.keyFrames(i).position.z;
        roll = dataSet.keyFrames(i).position.roll;
        pitch = dataSet.keyFrames(i).position.pitch;
        yaw = dataSet.keyFrames(i).position.yaw;
        poseSonar = transformationsMatrix(roll,pitch,yaw,xPos,yPos,zPos);
        localRotationIntensityRay = transformationsMatrix(0,0,dataSet.keyFrames(i).intensityValues.angle,0,0,0);
        
    
        ignoreDistanceIndex = round((0.5 / (dataSet.keyFrames(i).intensityValues.range))*(size(dataSet.keyFrames(i).intensityValues.intensity,1)));
    
    
        for j = ignoreDistanceIndex:size(dataSet.keyFrames(i).intensityValues.intensity,1)
    
            distanceOfIntensity = j / size(dataSet.keyFrames(i).intensityValues.intensity,1) *dataSet.keyFrames(i).intensityValues.range;
            % incrementOfScan = dataSet.keyFrames(i).intensityValues.intensity.range/size(dataSet.keyFrames(i).intensityValues.intensity,1);
            incrementOfScan = 1;
    
            for incrementTMP = -6:6% roll
                for incrementotherTMP = -30:30 % Y
                    positionOfIntensity = [distanceOfIntensity,0,0,1];
                    rotationOfPoint = incrementTMP / 400.0;
                    rotationForBetterView = transformationsMatrix(0,0,rotationOfPoint,0,0,0);
                    openingAngleMatrix = transformationsMatrix(0,incrementotherTMP/400.0,0,0,0,0);
                    positionOfIntensity = rotationForBetterView * openingAngleMatrix*positionOfIntensity';
                    positionOfIntensity = poseSonar * localRotationIntensityRay * positionOfIntensity;
                    indexX = round(positionOfIntensity(1) / (sizeOfMap / 2) * dimension3DMap /2 + dimension3DMap / 2)+1;
                    indexY = round(positionOfIntensity(2) / (sizeOfMap / 2) * dimension3DMap /2 +dimension3DMap / 2)+1;
                    if  indexX <= dimension3DMap && indexY <= dimension3DMap && indexY > 0 && indexX > 0
                        map3D(indexX,indexY,dimension3DMap/2) = map3D(indexX,indexY,dimension3DMap/2)+dataSet.keyFrames(i).intensityValues.intensity(j)*3.5;
                        map3DIndex(indexX,indexY,dimension3DMap/2) = map3DIndex(indexX,indexY,dimension3DMap/2)+1;
                    end
                end
            end
        end
    end
end
overallMean/numberOfMeanElements



for i = 1:dimension3DMap
    for j = 1:dimension3DMap
        for k = 1:dimension3DMap
            if(map3DIndex(i,j,k)>0)
                map3D(i,j,k) = map3D(i,j,k)/map3DIndex(i,j,k);
            end 
        end
    end
end



volumeViewer(map3D)









