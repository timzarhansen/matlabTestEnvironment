clc
clear

data = readJsonGraphPCL("testfile.json");


%% create datapoints for training %%
numberOfAdditionalPoints = 1;
sizeOfDataset = size(data,1)*(size(data,2)-1) * (1+numberOfAdditionalPoints);
dataset = zeros(sizeOfDataset,3);% occupancy x y 
currentAddedPoint = 0;
for i=1:size(data,1)
    %currentAddedPoint = currentAddedPoint+1;
    % calculate shift
    shift = squeeze(data(i,1,1:3));
    shiftX = shift(1);
    shiftY = shift(2);
    for j = 2:size(data(i,:,:),2)
        currentAddedPoint = currentAddedPoint+1;
        dataset(currentAddedPoint,1) = 1;
        dataset(currentAddedPoint,2) = shiftX + data(i,j,1);
        dataset(currentAddedPoint,3) = shiftY + data(i,j,2);
        xPosList = linspace(0,data(i,j,1),100);
        yPosList = linspace(0,data(i,j,2),100);
        randomNumberVector = randi(100,numberOfAdditionalPoints,1);
        for k = 1:size(randomNumberVector)
            currentAddedPoint = currentAddedPoint+1;
            dataset(currentAddedPoint,1) = -1;
            dataset(currentAddedPoint,2) = shiftX +xPosList(randomNumberVector(k));
            dataset(currentAddedPoint,3) = shiftY +yPosList(randomNumberVector(k));
        end
    end
end

%% add mean-point cloud to each datapoint
% standardDeviation = 0.3;
% numberOfTakenSamples = 10;
% datasetNew = zeros(size(dataset,1)*numberOfTakenSamples,3);
% for i = 1:size(dataset,1)
%     for j = 1:numberOfTakenSamples
%       randomXPos = dataset(i,2)+standardDeviation*randn(1,1);
%       randomYPos = dataset(i,3)+standardDeviation*randn(1,1);
%       datasetNew((i-1)*numberOfTakenSamples+j,2)=randomXPos;
%       datasetNew((i-1)*numberOfTakenSamples+j,3)=randomYPos;
%       datasetNew((i-1)*numberOfTakenSamples+j,1)=dataset(i,1);
%     end
% end
% dataset = datasetNew;
% sizeOfDataset = size(dataset,1);
%% Plot the datapoints
figure(1)
hold on
numberOccupiedPoints = 1;
numberNonOccupiedPoints = 1;
for i = 1:size(dataset,1)
    if dataset(i,1) >0.1
        plotOccupiedPoints(numberOccupiedPoints,1:2) = dataset(i,2:3);
        numberOccupiedPoints = numberOccupiedPoints+1;
    else
        plotNonOccupiedPoints(numberNonOccupiedPoints,1:2) = dataset(i,2:3);
        numberNonOccupiedPoints = numberNonOccupiedPoints+1;
    end
    
end
plot(plotOccupiedPoints(:,1),plotOccupiedPoints(:,2),'Marker','.','Color','r','LineStyle','none')
plot(plotNonOccupiedPoints(:,1),plotNonOccupiedPoints(:,2),'Marker','.','Color','b','LineStyle','none')

%% create sparse features, which map data points to the hilbert space %%
fromTo = 30;
numberOfInducingPointsPerDimension = 240;%120;
xPos = linspace(-fromTo,fromTo,numberOfInducingPointsPerDimension);
yPos = linspace(-fromTo,fromTo,numberOfInducingPointsPerDimension);
inducingPoints = zeros(numberOfInducingPointsPerDimension^2,2);
for i = 1:numberOfInducingPointsPerDimension
    for j = 1:numberOfInducingPointsPerDimension
        inducingPoints((i-1)*numberOfInducingPointsPerDimension+j,1) = xPos(i);
        inducingPoints((i-1)*numberOfInducingPointsPerDimension+j,2) = yPos(j);
    end
end

%% mappingBySparse([3,2],inducingPoints)
mappingBySparse([30,30],inducingPoints)
%%
numberOfFeatures = numberOfInducingPointsPerDimension^2;
%% calculate the current gradient of function R(w) = lambda1 * sum(w^T * w) + lambda2 * sum(|w1|+|w2|+...+|wn|)
w = zeros(numberOfFeatures,1);%randn(numberOfFeatures,1);
%testX = [3,1];
%testLabel = -1;
%gradientOfMapping(testX,sPoints,bPoints,w,testLabel)

%% train vector w to correctly predict the correct occupacy %%
stepSize = 0.01;

%sizeOfDataset = 3;
%dataset = [-1,0,0;1,2,0;1,-2,0];
gradient = zeros(numberOfFeatures,1);
movingAverage = 0.99;
for i = 1:2
    randomDataPermutation = randperm(sizeOfDataset);
    testSet = dataset(randomDataPermutation,:);
    for j = 1:sizeOfDataset
        testX = testSet(j,2:3);
        testLabel= testSet(j,1);
        gradient = gradient*movingAverage+(1-movingAverage)*gradientOfMapping(testX,inducingPoints,w,testLabel);
        norm(gradient)
        w = w-stepSize*gradient;
    end
    %disp('next Round')
    %i
end


%% calculate example Grid and print
fromTo = 30;
gridSize = 500;
scaling = gridSize/(2*fromTo);
gridmapForPrint = zeros(gridSize*gridSize,3);
for i=1:gridSize
    for j=1:gridSize
        xPos = i/scaling-fromTo;
        yPos = j/scaling-fromTo;
        gridmapForPrint((i-1)*(gridSize)+j,1)=xPos;
        gridmapForPrint((i-1)*(gridSize)+j,2)=yPos;
        gridmapForPrint((i-1)*(gridSize)+j,3)=calculateOccupancy([xPos,yPos],w,inducingPoints);
    end
end
%%
figure(2);
plot3(gridmapForPrint(:,1),gridmapForPrint(:,2),30*gridmapForPrint(:,3), '.')
axis equal
% figure(3);
% 
% pcshow(pointCloud)
%% plotting Occupancy Grid
figure(4);
clearvars plotOccupiedPoints plotNonOccupiedPoints plotUnsureOccupiedPoints
hold on
distanceToMean = 0.1;
numberOccupiedPoints = 1;
numberNonOccupiedPoints = 1;
numberUnsureOccupiedPoints = 1;
for i = 1:size(gridmapForPrint,1)
    if gridmapForPrint(i,3) > 0.5+distanceToMean
        plotOccupiedPoints(numberOccupiedPoints,1:2) = gridmapForPrint(i,1:2);
        numberOccupiedPoints = numberOccupiedPoints+1;
    else
        if gridmapForPrint(i,3) <0.5-distanceToMean
            plotNonOccupiedPoints(numberNonOccupiedPoints,1:2) = gridmapForPrint(i,1:2);
            numberNonOccupiedPoints = numberNonOccupiedPoints+1;
        else
            plotUnsureOccupiedPoints(numberUnsureOccupiedPoints,1:2) = gridmapForPrint(i,1:2);
            numberUnsureOccupiedPoints = numberUnsureOccupiedPoints+1;
        end
    end
    
end
plot(plotOccupiedPoints(:,1),plotOccupiedPoints(:,2),'Marker','.','Color','r','LineStyle','none')
plot(plotNonOccupiedPoints(:,1),plotNonOccupiedPoints(:,2),'Marker','.','Color','g','LineStyle','none')
plot(plotUnsureOccupiedPoints(:,1),plotUnsureOccupiedPoints(:,2),'Marker','.','Color','b','LineStyle','none')

%%
%[sortedDistances sortIndexes] = sort(gridmapForPrint(:,3));
%Arrange the data so that points close to the center
%use the blue end of the colormap, and points 
%close to the edge use the red end of the colormap.

%xs = gridmapForPrint((sortIndexes),1);
%ys = gridmapForPrint((sortIndexes),2);

%cmap = jet(length(gridmapForPrint((sortIndexes),1))); % Make 1000 colors.
%scatter(xs, ys, 10, cmap, 'filled')

%calculateOccupancy([0,0],w,sPoints,bPoints)
HuberLossGrad(1,3)
%% Functions %%


function output = mappingBySparse(pointOfInterest,inducingPoints)
    output = zeros(size(inducingPoints,1),1);
    for i=1:size(inducingPoints,1)
        r = sqrt((pointOfInterest-inducingPoints(i,:))*0.1*(pointOfInterest-inducingPoints(i,:))');
        if r>1
            output(i) = 0;
        else
            output(i) = (2+cos(2*pi*r)/3)*(1-r)+1/(2*pi)*sin(2*pi*r);
        end
    end
    %output = rescale(output);
end


function gradient = gradientOfMapping(x,inducingPoints,w,yStar)

    gradient = -yStar*mappingBySparse(x,inducingPoints)/(1+exp(yStar*w'*mappingBySparse(x ,inducingPoints)));
    lambda1 = 0.0001;
    lambda2 = 0.001;
    % Rdt = lambda1 * sum(w' * w) + lambda2 * sum(vecnorm(w,p,1))
    % Rdt = zeros(size(sPoints,1),1);
    for i = 1:size(gradient,1)
        Rdt =  lambda1*w(i)*w(i)+lambda2*HuberLossGrad(w(i),0.1); % 0.1 is the position where smoothing starts
        gradient(i) = gradient(i)+Rdt;
    end
end


function [ vG ] = HuberLossGrad( vX, paramMu )

vG = ((abs(vX) <= paramMu) .* (vX ./ paramMu)) + ((abs(vX) > paramMu) .* sign(vX));


end

function [ occupancy ] = calculateOccupancy( x ,w,inducingPoints )

occupancyNegative = 1/(1+exp(w'*mappingBySparse(x ,inducingPoints)));

occupancy = 1-occupancyNegative;
end



