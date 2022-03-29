clc
clear
load('datasetSonar.mat')


SonarDataRaw(:,2) = mod(SonarDataRaw(:,2)+pi,2*pi);


%% calculate position over time
beginningTimeOfCorrection = 1093455408.4;
% logTime (s) bottomVelX (m/s), bottomVelY (m/s), bottomVelZ (m/s), rangeBottom1, rangeBottom2, rangeBottom3, heading, pitch, roll
dvlData = readmatrix('datasetGenerated/dvlDataInterpolated.csv');
% time,roll (deg),pitch (deg),yaw (deg),rollVel (rad/s),pitchVel (rad/s),yawVel (rad/s),accelX (m/s2),accelY (m/s2),accelZ (m/s2)
imuData = readmatrix('datasetGenerated/IMUDataInterpolated.csv');
% 'time', 'latitude', 'longitude','status','altitude','geoide altitude','true course (deg)', 'magnetic course (deg)', 'vel (knots)', 'vel (km/h)'
gtData = readmatrix('datasetGenerated/GTDataInterpolated.csv');
%

timeStampsOfInterest = SonarDataRaw(:,1);
timeStampsOfInterest = timeStampsOfInterest(timeStampsOfInterest>beginningTimeOfCorrection);
timeStampsOfInterest=timeStampsOfInterest(1:end-10,:);%remove last 10 entries

%% calc only pos change from DVL
posDiff = zeros(size(timeStampsOfInterest,1)-1,3);
for i=1:size(timeStampsOfInterest,1)-1
    beginningIndex=1;
    while timeStampsOfInterest(i)>dvlData(beginningIndex,1)
        beginningIndex=beginningIndex+1;
    end
    beginningIndex=beginningIndex-1;

    endIndex = 1;
    while dvlData(endIndex,1)<timeStampsOfInterest(i+1)
        endIndex=endIndex+1;
    end
    % endIndex=endIndex+1;
    currentTranslation = [0 0 0];
    if(beginningIndex+1==endIndex)
        % somewhere inbetween
        interpolationFactor = (timeStampsOfInterest(i+1)-timeStampsOfInterest(i))/(dvlData(endIndex,1)-dvlData(beginningIndex,1));
        currentTranslation =currentTranslation+interpolationFactor*(dvlData(endIndex,1)-dvlData(beginningIndex,1))*[(dvlData(beginningIndex,2)+dvlData(endIndex+1,2))/2 (dvlData(beginningIndex,3)+dvlData(endIndex+1,3))/2 0];

    else

    
        %interpolate beginning factor to multiply with change in position
        %interpolationFactorBeginning = 1-(timeStampsOfInterest(i)-dvlData(beginningIndex,1))/(dvlData(beginningIndex+1,1)-dvlData(beginningIndex,1));
        currentTranslation = currentTranslation+(timeStampsOfInterest(i)-dvlData(beginningIndex,1))*[(dvlData(beginningIndex,2)+dvlData(beginningIndex+1,2))/2 (dvlData(beginningIndex,3)+dvlData(beginningIndex+1,3))/2 0];
    
    
    
        %add everything inbetween
        if(beginningIndex+2==endIndex)
            %no need to do anything
        else
            for k = beginningIndex+1:endIndex-1
                currentTranslation = currentTranslation+(dvlData(k+1,1)-dvlData(k,1))*[(dvlData(k,2)+dvlData(k+1,2))/2 (dvlData(k,3)+dvlData(k+1,3))/2 0];
            end
    
        end


        %interpolate end
        %interpolationFactorEnd = (dvlData(endIndex,1)-timeStampsOfInterest(i+1))/(dvlData(endIndex,1)-dvlData(endIndex-1,1));
        currentTranslation = currentTranslation+(timeStampsOfInterest(i+1)-dvlData(endIndex-1,1))*[(dvlData(endIndex,2)+dvlData(endIndex-1,2))/2 (dvlData(endIndex,3)+dvlData(endIndex-1,3))/2 0];

    end
    posDiff(i,1:3) = currentTranslation;



end
%% angle diff calc
angleDiff = zeros(size(timeStampsOfInterest,1)-1,1);
for i=1:size(timeStampsOfInterest,1)-1
    beginningIndex=1;
    while timeStampsOfInterest(i)>gtData(beginningIndex,1)
        beginningIndex=beginningIndex+1;
    end
    beginningIndex=beginningIndex-1;

    endIndex = 1;
    while gtData(endIndex,1)<timeStampsOfInterest(i+1)
        endIndex=endIndex+1;
    end
    % endIndex=endIndex+1;
    angleDiffTMP = 0;
    if(beginningIndex+1==endIndex)
        % somewhere inbetween
        interpolationFactor = (timeStampsOfInterest(i+1)-timeStampsOfInterest(i))/(gtData(endIndex,1)-gtData(beginningIndex,1));
        angleDiffTMP = angleDiffTMP+interpolationFactor*(gtData(beginningIndex+1,7)-gtData(beginningIndex,7));

    else

    
        %interpolate beginning factor to multiply with change in position
        %interpolationFactorBeginning = 1-(timeStampsOfInterest(i)-dvlData(beginningIndex,1))/(dvlData(beginningIndex+1,1)-dvlData(beginningIndex,1));
        angleDiffTMP = angleDiffTMP+(timeStampsOfInterest(i)-gtData(beginningIndex,1))*(gtData(beginningIndex+1,7)-gtData(beginningIndex,7));
    
    
    
        %add everything inbetween
        if(beginningIndex+2==endIndex)
            %no need to do anything
        else
            for k = beginningIndex+1:endIndex-1
                angleDiffTMP = angleDiffTMP+(gtData(k+1,1)-gtData(k,1))*(gtData(k,7)+gtData(k+1,7)) ;
            end
    
        end


        %interpolate end
        %interpolationFactorEnd = (dvlData(endIndex,1)-timeStampsOfInterest(i+1))/(dvlData(endIndex,1)-dvlData(endIndex-1,1));
        %currentTranslation = currentTranslation+(timeStampsOfInterest(i+1)-dvlData(endIndex-1,1))*[(dvlData(endIndex,2)+dvlData(endIndex-1,2))/2 (dvlData(endIndex,3)+dvlData(endIndex-1,3))/2 0];
        angleDiffTMP = angleDiffTMP+(timeStampsOfInterest(i+1)-gtData(endIndex-1,1))*(gtData(endIndex,7)-gtData(endIndex-1,7));
    end
    angleDiff(i) = angleDiffTMP;



end
angleDiff=angleDiff/400*2*pi;
%%





%find first scan
for i = 1:size(SonarDataRaw,1)
    if(abs(SonarDataRaw(i,2))<0.05)
        break
    end
end

startFirstScan=i;

currentStart = startFirstScan;

%%
eraseFirstNentries = 10;%erases the first 50 cm of the bins

j = 1;
while size(SonarDataRaw,1)-currentStart>200
    beginningTimeStamp = SonarDataRaw(currentStart,1);
    bins(1,1:500) = SonarDataRaw(currentStart,3:502);
    angle(1) = SonarDataRaw(currentStart,2);
    i=1;
    while abs(SonarDataRaw(currentStart+i,2))>0.05
        bins(i,1:500) = SonarDataRaw(currentStart+i,3:502);
        angle(i)=SonarDataRaw(currentStart+i,2);
        currentTransformationTMP=eye(3);
        k=0;
        while abs(SonarDataRaw(currentStart+i+k,2))>0.05
            rotation3D = rotationMatrix(0,0,angleDiff(currentStart+i+k));
            currentTransformationTMP(1:2,1:2) = currentTransformationTMP(1:2,1:2)*rotation3D(1:2,1:2);
            posDiff3D = posDiff(currentStart+i+k,:);
            currentTransformationTMP(1:2,3)= currentTransformationTMP(1:2,3)+posDiff3D(1:2)';

            k=k+1;
        end
        currentTransformationToEnd(i,:,:)=currentTransformationTMP;
        i=i+1;

    end
    currentTransformationToEnd(i,:,:)=eye(3);



    bins(i,1:500) = SonarDataRaw(currentStart+i,3:502);
    angle(i)=SonarDataRaw(currentStart+i,2);
    endTimeStamp = SonarDataRaw(currentStart+i,1);
    currentStart = currentStart+i;
    % save current pcl

    positionMatrix = linspace(0.1,50,500).*ones(size(angle,2),1);
    positionMatrix = positionMatrix(:,eraseFirstNentries:end);
    x=positionMatrix.*cos(angle');
    y=positionMatrix.*sin(angle');

    for k=1:size(x,1)
        correctTranform = pagemtimes(squeeze(currentTransformationToEnd(k,:,:)),[x(k,:);y(k,:);linspace(1,1,size(x,2))]);
        actualx(k,:) = correctTranform(1,:);
        actualy(k,:) = correctTranform(2,:);
    end


    [pcl,binsCleaned]=binsToPCL(bins,actualx,actualy,eraseFirstNentries);
    figure(1)
    h = surf(x,y,binsCleaned);
    set(h,'LineStyle','none')
    colormap jet
    axis equal
    view(2)
    figure(2)
    folderName = "correctedDataset";
    %folderName = "datasetGenerated";
    pcwrite(pcl,folderName+"/pclKeyFrame"+j+"TMP.pcd")
    system("pdal translate "+folderName+"/pclKeyFrame"+j+"TMP.pcd "+folderName+"/pclKeyFrame"+j+".pcd --writers.pcd.order=""x=Float:4,y=Float:4,z=Float:4""")
    system("sed -i 's/FIELDS X Y Z/FIELDS x y z/g' "+folderName+"/pclKeyFrame"+j+".pcd")
    system("rm "+folderName+"/pclKeyFrame"+j+"TMP.pcd")
    pcshow(pcl)
    savingKeyframeTimeStamps(j,1:3)=[j,beginningTimeStamp,endTimeStamp];
    j=j+1;
    
    
end

savingKeyframeTimeStampsTable = array2table(savingKeyframeTimeStamps);
savingKeyframeTimeStampsTable.Properties.VariableNames(1:3) = {'keyFrame', 'keyFrameBeginning', 'keyFrameEnd'};
writetable(savingKeyframeTimeStampsTable,'datasetGenerated/keyFrameTimeStamps.csv')


