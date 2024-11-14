function [outputArg1,outputArg2] = plotPDFs(nameOfTheDataset,nameOfBatchFile,nameOfTheTitelList,nameOfResultFile)
%PLOTPDFS Summary of this function goes here
%   Detailed explanation goes here
    groundTruthRaw = readmatrix("csvFiles/groundTruthOverTime"+nameOfTheDataset+".csv");
    positionEstimateEKFRaw = readmatrix("csvFiles/positionEstimationOverTime"+nameOfTheDataset+".csv");
    angleAndIntensitiesRaw = readmatrix("csvFiles/angleAndIntensities"+nameOfTheDataset+".csv");
    numberOfMarkers = readmatrix("csvFiles/numberOfMarkers"+nameOfTheDataset+".csv");
    nameOfTheDataset;
    size(angleAndIntensitiesRaw);
    positionFirstGTMatrix = 3;
    while(isnan(groundTruthRaw(4*positionFirstGTMatrix+1,1)))
        positionFirstGTMatrix=positionFirstGTMatrix+1;
    end
    
    
    
    numberOfSonarMeasurements = size(groundTruthRaw,1)/4;
    % first position setting equal matrix:
    firstGTMatrix = groundTruthRaw(4*positionFirstGTMatrix+1:4*positionFirstGTMatrix+4,1:4);
    estimateEKFFirst = positionEstimateEKFRaw(4*positionFirstGTMatrix+1:4*positionFirstGTMatrix+4,1:4);
    changeOfEKFMatrix = firstGTMatrix*inv(estimateEKFFirst);
    
    for i = 1:numberOfSonarMeasurements
        xGT(i) = groundTruthRaw((i-1)*4+1,4);
        yGT(i) = groundTruthRaw((i-1)*4+2,4);
        tmpEKFMatrix = changeOfEKFMatrix*positionEstimateEKFRaw((i-1)*4+1:(i-1)*4+4,1:4);
        xEKF(i) = tmpEKFMatrix(1,4);
        yEKF(i) = tmpEKFMatrix(2,4);
        rotationMatrixEKF(i,1:3,1:3) = tmpEKFMatrix(1:3,1:3);
        rotationMatrixEGT(i,1:3,1:3) = groundTruthRaw((i-1)*4+1:(i-1)*4+3,1:3);




    end
    

    
    j = 1;


    for i = 2:numberOfSonarMeasurements
        if (~isnan(xGT(i)) & ~isnan(yGT(i)) & ~isnan(xEKF(i)) & ~isnan(yEKF(i)) & ~isnan(rotationMatrixEKF(i,1:3,1:3)) & ~isnan(rotationMatrixEGT(i,1:3,1:3)) )
            % do computation 
            errorRotationMatrix = squeeze(rotationMatrixEKF(i,1:3,1:3))*inv(squeeze(rotationMatrixEGT(i,1:3,1:3)));
            yawError(j) = atan2(errorRotationMatrix(2,1),errorRotationMatrix(1,1));
            errorX(j) = xGT(i)-xEKF(i);
            errorY(j) = yGT(i)-yEKF(i);
            normError(j) = sqrt(errorX(j)^2 + errorY(j)^2);
            j = j+1;
        end

    end
    
    display(nameOfTheDataset)
    display(mean(abs(errorX)))
    display(std(abs(errorX)))
    display(mean(abs(errorY)))
    display(std(abs(errorY)))
    display(mean(abs(normError)))
    display(std(abs(normError)))
    display(mean(abs(yawError)))
    display(std(abs(yawError)))
    display(normError(end))


    figure(1)
    clf
    % subplot(2,1,1)
    hold on
    plot(xGT,yGT)
    
    plot(xEKF,yEKF)
    
    legend('NaN silently ignored','Location','northwest')
    axis equal
    % subplot(2,1,2)
    % hold on
    % 
    % plot(xGT(isfinite(yGT)),yGT(isfinite(yGT))) % use the same indexing for both x & y
    % plot(xEKF(isfinite(yEKF)),yEKF(isfinite(yEKF)))
    % legend('NaN removed from dataset','Location','northwest')
    % axis equal






    
    %%
    
    figure(2)
    clf
    %plot the image
    numberOfPixels = 256;
    sizeOfMap = 45;
    sizeOfCell = sizeOfMap/numberOfPixels;
    map = zeros(numberOfPixels);
    mapIndex = zeros(numberOfPixels);
    
    for i = positionFirstGTMatrix:numberOfSonarMeasurements
        currentSonarMeasurement = angleAndIntensitiesRaw(i,:);
        angleSonar = currentSonarMeasurement(2);
        angleSonarMatrixTMP = rotationMatrix(0,0,0)*rotationMatrix(0,0,angleSonar);
        angleSonarMatrix = eye(4);
        angleSonarMatrix(1:3,1:3) = angleSonarMatrixTMP;
        range = currentSonarMeasurement(1);
        positionRobot = firstGTMatrix*inv(firstGTMatrix)*groundTruthRaw((i-1)*4+1:(i-1)*4+4,1:4);
        % positionRobot = inv(firstGTMatrix)*changeOfEKFMatrix*positionEstimateEKFRaw((i-1)*4+1:(i-1)*4+4,1:4);
    
        rotationMatrix90Degree = eye(4);
        rotationMatrix90Degree(1:3,1:3) = rotationMatrix(pi,0,0)*rotationMatrix(0,0,-pi/2);
        positionRobotSave(i,1:4,1:4) = rotationMatrix90Degree*positionRobot;
        % angleOfRobot = atan2(positionRobotInPixel(2,1),positionRobotInPixel(1,1));
    
        % angleSonarReal = angleSonar + angleOfRobot;
        if ~isnan(positionRobot(1,1))
        
            numberOfIntensityValues = size(currentSonarMeasurement,2)-2;
            for j=10:numberOfIntensityValues 
                % display(j*range/numberOfIntensityValues)
                positionOfIntensity = positionRobot*angleSonarMatrix*[j*range/numberOfIntensityValues 0 0 1]';
                positionInPixel = positionOfIntensity/sizeOfCell;
                % needs to calculate in pixel values
                xPosInMap = round(positionInPixel(1));
                yPosInMap = round(positionInPixel(2));
                if j == 10
                    % display(xPosInMap)
                    % display(positionRobot(1,4)/sizeOfCell)
                    % display(yPosInMap)
                    % display(positionRobot(2,4)/sizeOfCell)
                end
                xIndexMap = round((xPosInMap)+numberOfPixels / 2);
                yIndexMap = round((yPosInMap)+numberOfPixels / 2);
                if(xIndexMap<numberOfPixels && yIndexMap<numberOfPixels&&yIndexMap>0&&xIndexMap>0)
                    map(xIndexMap,yIndexMap) = map(xIndexMap,yIndexMap)+currentSonarMeasurement(1,j+2);
                    mapIndex(xIndexMap,yIndexMap) = mapIndex(xIndexMap,yIndexMap)+1;
                end
    
            end
        end
        
    end
    
    maxMap = 0;
    for i = 1:numberOfPixels
        for j = 1:numberOfPixels
            if(mapIndex(i,j)>0)    
                map(i,j)=map(i,j)/mapIndex(i,j);
                if(maxMap<map(i,j))
                    maxMap = map(i,j);
                end
            end
            
        end
    end
    
    
    for i = 1:numberOfPixels
        for j = 1:numberOfPixels
            map(i,j)=map(i,j)/maxMap;
        end
    end
    
    ax1 = axes;
    
    imagesc(ax1,linspace(-sizeOfMap/2,sizeOfMap/2,numberOfPixels),linspace(-sizeOfMap/2,sizeOfMap/2,numberOfPixels), map);

    % Plot first data 
    hold on;

    % plotting positions Of Cameras
	
    positionsOfCameras = [[-8.54974;8.68527],[-8.49616;4.99751],[-8.39248;-3.68944],[2.08273;-6.38497],[-2.46475;-6.44979],[-7.45200;-6.50915],[9.57199;-5.44995],[9.50768;0.69213],[9.55034;-3.12613],[9.45405;3.97318],[-8.43350;0.45310],[9.38556;8.88193]]';
	for i = 1:size(positionsOfCameras,1)
        currentPositionCameras = positionsOfCameras(i,:);
        positionsOfCameras(i,1) = currentPositionCameras(2);
        positionsOfCameras(i,2) = currentPositionCameras(1);
    end
    plot(positionsOfCameras(:,1), positionsOfCameras(:,2), 'x','color',"red",'LineWidth',2)

	


    % Plot second data 
    ax2 = axes; 
    %ax2.YDir = 'reverse';
    % scatter(ax2, x, y, [], z, 'd', 'filled');
    
    c = numberOfMarkers;%340*(numberOfMarkers - min(numberOfMarkers)) / ( max(numberOfMarkers) - min(numberOfMarkers) );
    
    % scatter(ax2,positionRobotSave(:,1,4)/sizeOfCell+numberOfPixels / 2,positionRobotSave(:,2,4)/sizeOfCell+numberOfPixels / 2,4,c,"filled")
    scatter(ax2,positionRobotSave(:,1,4),positionRobotSave(:,2,4),4,c,"filled")

    tmpXTick = ax2.XTick;
    tmpYTick = ax2.YTick;
    limXTick = ax2.XLim;
    
    % Link axes 
    linkaxes([ax1, ax2]);
    % Hide the top axes 
    ax2.Visible = 'off';
    %ax2.XTick = []; 
    %ax2.YTick = [];
    
    ax2.YDir = 'reverse';
    % Add differenct colormap to different data if you wish 
    colormap(ax1, 'jet') 
    colormap(ax2, 'hot') 
    % Set the axes and colorbar position 
    set([ax1,ax2],'Position', [.17 .11 .685 .815]); 
    cb1 = colorbar(ax1,'Position', [.05 .11 .0675 .815]); 
    cb2 = colorbar(ax2,'Position', [.88 .11 .0675 .815]);


    hold off;
    
    % ax1.XTick = tmpXTick;
    % ax1.YTick = tmpYTick;
    % ax1.XTick = tmpXTick
    % for i = 1:size(tmpXTick,2)
    %     myXTicks(i) = (tmpXTick(1,i));
    % end
    % ax = gca
    % tmp = convertStringsToChars(string(myXTicks))
    % xticklabels(tmp);
    % ax1.XTick = linspace(ax1.XLim(1),ax1.XLim(2),size(tmpXTick,2));%1:size(tmpXTick,2);
    % ax1.XTickLabel = tmpXTick(1,:);
    
    pbaspect(ax1,[1 1 1])
    pbaspect(ax2,[1 1 1])
    title(ax1,'Dataset: '+nameOfTheTitelList)

    nameOfPdfFile = '/home/ws/matlab/registrationFourier/datasetReleaseJournal/PDFS/GTMap' +string(nameOfTheDataset);
    saveas(gcf,nameOfPdfFile, 'pdf' )
    addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf &',true);

%%

    figure(3)
    clf

    map = zeros(numberOfPixels);
    mapIndex = zeros(numberOfPixels);
    
    for i = positionFirstGTMatrix:numberOfSonarMeasurements
        currentSonarMeasurement = angleAndIntensitiesRaw(i,:);
        angleSonar = currentSonarMeasurement(2);
        angleSonarMatrixTMP = rotationMatrix(0,0,0)*rotationMatrix(0,0,angleSonar);
        angleSonarMatrix = eye(4);
        angleSonarMatrix(1:3,1:3) = angleSonarMatrixTMP;
        range = currentSonarMeasurement(1);
        % positionRobotGT = firstGTMatrix*inv(firstGTMatrix)*groundTruthRaw((i-1)*4+1:(i-1)*4+4,1:4);
        positionRobot = firstGTMatrix*inv(firstGTMatrix)*changeOfEKFMatrix*positionEstimateEKFRaw((i-1)*4+1:(i-1)*4+4,1:4);
    
        rotationMatrix90Degree = eye(4);
        rotationMatrix90Degree(1:3,1:3) = rotationMatrix(pi,0,0)*rotationMatrix(0,0,-pi/2);
        positionRobotSave(i,1:4,1:4) = rotationMatrix90Degree*positionRobot;
        % angleOfRobot = atan2(positionRobotInPixel(2,1),positionRobotInPixel(1,1));
    
        % angleSonarReal = angleSonar + angleOfRobot;
        if ~isnan(positionRobot(1,1))
        
            numberOfIntensityValues = size(currentSonarMeasurement,2)-2;
            for j=10:numberOfIntensityValues 
                % display(j*range/numberOfIntensityValues)
                positionOfIntensity = positionRobot*angleSonarMatrix*[j*range/numberOfIntensityValues 0 0 1]';
                positionInPixel = positionOfIntensity/sizeOfCell;
                % needs to calculate in pixel values
                xPosInMap = round(positionInPixel(1));
                yPosInMap = round(positionInPixel(2));
                if j == 10
                    % display(xPosInMap)
                    % display(positionRobot(1,4)/sizeOfCell)
                    % display(yPosInMap)
                    % display(positionRobot(2,4)/sizeOfCell)
                end
                xIndexMap = round((xPosInMap)+numberOfPixels / 2);
                yIndexMap = round((yPosInMap)+numberOfPixels / 2);
                if(xIndexMap<numberOfPixels && yIndexMap<numberOfPixels&&yIndexMap>0&&xIndexMap>0)
                    map(xIndexMap,yIndexMap) = map(xIndexMap,yIndexMap)+currentSonarMeasurement(1,j+2);
                    mapIndex(xIndexMap,yIndexMap) = mapIndex(xIndexMap,yIndexMap)+1;
                end
    
            end
        end
        
    end
    
    
    maxMap = 0;
    for i = 1:numberOfPixels
        for j = 1:numberOfPixels
            if(mapIndex(i,j)>0)    
                map(i,j)=map(i,j)/mapIndex(i,j);
                if(maxMap<map(i,j))
                    maxMap = map(i,j);
                end
            end
            
        end
    end
    
    
    for i = 1:numberOfPixels
        for j = 1:numberOfPixels
            map(i,j)=map(i,j)/maxMap;
        end
    end
    
    
    
    
    ax1 = axes;
    
    imagesc(ax1,linspace(-sizeOfMap/2,sizeOfMap/2,numberOfPixels),linspace(-sizeOfMap/2,sizeOfMap/2,numberOfPixels), map);

    % Plot first data 
    hold on;
    % Plot second data 
    ax2 = axes; 
    %ax2.YDir = 'reverse';
    % scatter(ax2, x, y, [], z, 'd', 'filled');
    
    c = numberOfMarkers;%340*(numberOfMarkers - min(numberOfMarkers)) / ( max(numberOfMarkers) - min(numberOfMarkers) );
    
    % scatter(ax2,positionRobotSave(:,1,4)/sizeOfCell+numberOfPixels / 2,positionRobotSave(:,2,4)/sizeOfCell+numberOfPixels / 2,4,c,"filled")
    scatter(ax2,positionRobotSave(:,1,4),positionRobotSave(:,2,4),4,c,"filled")

    tmpXTick = ax2.XTick;
    tmpYTick = ax2.YTick;
    limXTick = ax2.XLim;
    
    % Link axes 
    linkaxes([ax1, ax2]);
    % Hide the top axes 
    ax2.Visible = 'off';
    %ax2.XTick = []; 
    %ax2.YTick = [];
    
    ax2.YDir = 'reverse';
    % Add differenct colormap to different data if you wish 
    lvl = [1:10];
    % interval/category colors
    CT0 = jet(numel(lvl)-1);
    % generate expanded color table & info
    
    [CT cidx] = intervalct(lvl,CT0,10);

    colormap(ax1, 'jet') 
    colormap(ax2, 'hot') 
    % Set the axes and colorbar position 
    % set([ax1,ax2],'Position', [.17 .11 .685 .815]); 
    cb1 = colorbar(ax1,'Position', [.05 .11 .0675 .815]); 
    cb2 = colorbar(ax2,'Position', [.88 .11 .0675 .815]);
    hold off;
    
    % ax1.XTick = tmpXTick;
    % ax1.YTick = tmpYTick;
    % ax1.XTick = tmpXTick
    % for i = 1:size(tmpXTick,2)
    %     myXTicks(i) = (tmpXTick(1,i));
    % end
    % ax = gca
    % tmp = convertStringsToChars(string(myXTicks))
    % xticklabels(tmp);
    % ax1.XTick = linspace(ax1.XLim(1),ax1.XLim(2),size(tmpXTick,2));%1:size(tmpXTick,2);
    % ax1.XTickLabel = tmpXTick(1,:);
    
    pbaspect(ax1,[1 1 1])
    pbaspect(ax2,[1 1 1])
    title(ax1,'Dataset: '+nameOfTheTitelList)

    nameOfPdfFile = '/home/ws/matlab/registrationFourier/datasetReleaseJournal/PDFS/EKFMap' +string(nameOfTheDataset);
    saveas(gcf,nameOfPdfFile, 'pdf' )
    addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf &',true);



end

