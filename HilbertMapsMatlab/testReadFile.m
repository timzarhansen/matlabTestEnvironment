clc
clear
data = readJsonGraphPCL("testfile.json");


%% create datapoints for training %%
numberOfAdditionalPoints = 4;
sizeOfDataset = size(data,1)*size(data,2) * (1+numberOfAdditionalPoints);
dataset = zeros(sizeOfDataset,3);% occupancy x y 
currentAddedPoint = 0;
for i=1:size(data,1)
    currentAddedPoint = currentAddedPoint+1;
    % calculate shift
    shift = squeeze(data(i,1,1:3));
    shiftX = shift(1);
    shiftY = shift(2);
    for j = 2:size(data(i,:,:),2)
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