clc
clear

pointCloud = pcread("after_voxel_second.pcd");


%% create datapoints for training %%
numberOfAdditionalPoints = 8;
sizeOfDataset = pointCloud.Count * (1+numberOfAdditionalPoints);
dataset = zeros(sizeOfDataset,3);% occupancy x y 
currentAddedPoint = 0;
for i=1:pointCloud.Count
    currentAddedPoint = currentAddedPoint+1;
    dataset(currentAddedPoint,1) = 1;
    dataset(currentAddedPoint,2) = pointCloud.Location(i,1);
    dataset(currentAddedPoint,3) = pointCloud.Location(i,2);
    xPosList = linspace(0,pointCloud.Location(i,1),100);
    yPosList = linspace(0,pointCloud.Location(i,2),100);
    randomNumberVector = randi(100,numberOfAdditionalPoints,1);
    for k = 1:size(randomNumberVector)
        currentAddedPoint = currentAddedPoint+1;
        dataset(currentAddedPoint,1) = -1;
        dataset(currentAddedPoint,2) = xPosList(randomNumberVector(k));
        dataset(currentAddedPoint,3) = yPosList(randomNumberVector(k));
    end 
end

%% add mean-point cloud to each datapoint
standardDeviation = 2;
numberOfTakenSamples = 20;
datasetNew = zeros(size(dataset,1)*numberOfTakenSamples,3);
for i = 1:size(dataset,1)
    for j = 1:numberOfTakenSamples
      randomXPos = dataset(i,2)+standardDeviation*randn(1,1);
      randomYPos = dataset(i,3)+standardDeviation*randn(1,1);
      datasetNew((i-1)*numberOfTakenSamples+j,2)=randomXPos;
      datasetNew((i-1)*numberOfTakenSamples+j,3)=randomYPos;
      datasetNew((i-1)*numberOfTakenSamples+j,1)=dataset(i,1);
    end
end
dataset = datasetNew;
sizeOfDataset = size(dataset,1);


%% create fourie features, which map data points to the hilbert space %%
numberOfFeatures = 10000;
sigma = 1 ;

sPoints = 2*sigma^(-2)*randn(numberOfFeatures,2);

bPoints = 2*pi*rand(numberOfFeatures,1);


%mappingByFourier([3,2],sPoints,bPoints)


%% calculate the current gradient of function R(w) = lambda1 * sum(w^T * w) + lambda2 * sum(|w1|+|w2|+...+|wn|)
w = randn(numberOfFeatures,1);
%testX = [3,1];
%testLabel = -1;
%gradientOfMapping(testX,sPoints,bPoints,w,testLabel)

%% train vector w to correctly predict the correct occupacy %%
stepSize = 0.01;

%sizeOfDataset = 3;
%dataset = [-1,0,0;1,2,0;1,-2,0];
gradient = zeros(numberOfFeatures,1);
movingAverage = 0.99;
for i = 1:5
    randomDataPermutation = randperm(sizeOfDataset);
    testSet = dataset(randomDataPermutation,:);
    for j = 1:sizeOfDataset
        testX = testSet(j,2:3);
        testLabel= testSet(j,1);
        gradient = gradient*movingAverage+(1-movingAverage)*gradientOfMapping(testX,sPoints,bPoints,w,testLabel);
        %norm(gradient)
        w = w-stepSize*gradient;
    end
    %disp('next Round')
    %i
end


%% calculate example Grid and print
fromTo = 30;
gridSize = 100;
scaling = gridSize/(2*fromTo);
gridmapForPrint = zeros(gridSize*gridSize,3);
for i=1:gridSize
    for j=1:gridSize
        xPos = i/scaling-fromTo;
        yPos = j/scaling-fromTo;
        gridmapForPrint((i-1)*(gridSize)+j,1)=xPos;
        gridmapForPrint((i-1)*(gridSize)+j,2)=yPos;
        gridmapForPrint((i-1)*(gridSize)+j,3)=calculateOccupancy([xPos,yPos],w,sPoints,bPoints);
    end
end
%%
figure;
plot3(gridmapForPrint(:,1),gridmapForPrint(:,2),30*gridmapForPrint(:,3), '.')
axis equal
figure(2);

pcshow(pointCloud)
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

%% Functions %%



function output = mappingByFourier(x,sPoints,bPoints)
    output = zeros(size(sPoints,1),1);
    for i = 1:size(output,1)
        output(i,1) = cos(sPoints(i,1:2)*x'+bPoints(i));
    end
    
    output = output/sqrt(size(sPoints,1));
    
    output = rescale(output);
end

function gradient = gradientOfMapping(x,sPoints,bPoints,w,yStar)

    gradient = -yStar*mappingByFourier(x,sPoints,bPoints)/(1+exp(yStar*w'*mappingByFourier(x,sPoints,bPoints)));
    lambda1 = 0.0001;
    lambda2 = 0.15;
    % Rdt = lambda1 * sum(w' * w) + lambda2 * sum(vecnorm(w,p,1))
    % Rdt = zeros(size(sPoints,1),1);
    for i = 1:size(gradient,1)
        Rdt =  lambda1*2*w(i)+lambda2*HuberLossGrad(w(i),0.1); % 0.1 is the position where smoothing starts
        gradient(i) = gradient(i)+Rdt;
    end
end


function [ vG ] = HuberLossGrad( vX, paramMu )

vG = ((abs(vX) <= paramMu) .* (vX ./ paramMu)) + ((abs(vX) > paramMu) .* sign(vX));


end

function [ occupancy ] = calculateOccupancy( x ,w,sPoints,bPoints )

occupancyNegative = 1/(1+exp(w'*mappingByFourier(x ,sPoints,bPoints)));

occupancy = 1-occupancyNegative;
end


