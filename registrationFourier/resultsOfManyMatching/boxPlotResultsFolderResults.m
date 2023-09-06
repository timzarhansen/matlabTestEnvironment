clc
clear

%1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 
%5: FourierMellinTransform(oneSize)(missing)
%6: Our 32 FMS 2D, 7: Our 64 FMS 2D, 8: Our 128 FMS 2D, 9: Our 256 FMS 2D
% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/test/";
% whichDataset = "test";
%% Valentin:
% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/valentinNoNoise52/";
% whichDataset = "valentinBunkerNoNoise52";

% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/valentinHighNoise52/";
% whichDataset = "valentinBunkerHighNoise52";


% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/valentinNoNoise1510/";
% whichDataset = "valentinBunkerNoNoise1510";

% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/valentinHighNoise52NEW/";
% whichDataset = "valentinHighNoise52NEW";

%% StPere:

% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/stPereNoNoise52/";
% whichDataset = "StPereNoNoise52";

% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/stPereHighNoise52/";
% whichDataset = "StPereHighNoise52";

% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/stPereHighNoise52/";
% whichDataset = "StPereHighNoise52NEW";

% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/stPereNoNoise1510/";
% whichDataset = "StPereNoNoise1510";

%% Simulation


% whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/SimulationResults/lowNoise305_52/";
% whichDataset = "consecutive";

%% test

whichDatasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/valentinNoNoise1510NEW/";
whichDataset = "valentinNoNoise1510NEW";


%%
threshold = 1;
%whichError = "1510";
timeMatrix = readmatrix(whichDatasetFolder+ "calculationTime"+ string(threshold) +".csv");
errorAngleMatrix = readmatrix(whichDatasetFolder +"errorAngle"+ string(threshold) +".csv");
errorDistanceMatrix = readmatrix(whichDatasetFolder +"errorDistance"+ string(threshold) +".csv");




%% calculate mean std div and median of computation time error angle and l2 norm error

currentMethod =["GICP", "Super4PCS",  "NDT D2D 2D",  "NDT P2D", "Our 2D FMS 32",  "Our 2D FMS 64", "Our 2D FMS 128", "Our FMS 2D 256", "Our Global 2D FMS 32",  "Our Global 2D FMS 64", "Our Global 2D FMS 128", "Our Global FMS 2D 256", "no registration"];


for i = 1:13
    meanCalculationTime(i) = mean(timeMatrix(:,i))*0.001;
    stdCalculationTime(i) = std(timeMatrix(:,i))*0.001;

    display(currentMethod(i)+" meanTime: "+meanCalculationTime(i))
    display(currentMethod(i)+" stdDivTime: "+stdCalculationTime(i))
    %display(currentMethod(i)+" medianTime: "+median(timeMatrix(:,i))*0.001)
end

for i = 1:13
%      = mean(errorAngleMatrix(:,i))*180/pi;
%     = std(errorAngleMatrix(:,i))*180/pi;


    angleTMP = errorAngleMatrix(:,i);
    angleTMP((angleTMP>10000))=0;
%     angleTMP((angleTMP>0.9))=0;
    angleTMP = nonzeros(angleTMP);
    angleTMP = angleTMP(~isnan(angleTMP));
    meanAngle(i) = mean(angleTMP)*180/pi;
    stdAngle(i) = std(angleTMP)*180/pi;




    display(currentMethod(i)+" meanAngle: " + meanAngle(i))
    display(currentMethod(i)+" stdDivAngle: " + stdAngle(i))
    %display(currentMethod(i)+" medianAngle: "+median(errorAngleMatrix(:,i))*180/pi)
end

for i = 1:13

    distTMP = errorDistanceMatrix(:,i);
    distTMP((distTMP>100000))=0;
    distTMP = nonzeros(distTMP);
    distTMP = distTMP(~isnan(distTMP));
    meanDistance(i) = mean(distTMP);
    stdDistance(i) = std(distTMP);

    display(currentMethod(i)+" meanl2Norm: "+meanDistance(i))
    display(currentMethod(i)+" stdDivl2Norm: "+stdDistance(i))
    %display(currentMethod(i)+" medianl2Norm: "+median(errorDistanceMatrix(:,i)))
end


%%
f = figure(4);
boxplot(errorAngleMatrix,"Whisker",1);
ax = gca;
ax.YAxis.Scale ="log";
xticklabels({"GICP "+num2str(meanAngle(1),4)+"+-"+num2str(stdAngle(1),4), ...
    "Super4PCS "+num2str(meanAngle(2),4)+"+-"+num2str(stdAngle(2),4), ...
    "NDT D2D 2D "+num2str(meanAngle(3),4)+"+-"+num2str(stdAngle(3),4), ...
    "NDT P2D "+num2str(meanAngle(4),4)+"+-"+num2str(stdAngle(4),4), ...
    "Our 2D FMS 32 "+num2str(meanAngle(5),4)+"+-"+num2str(stdAngle(5),4), ...
    "Our 2D FMS 64 "+num2str(meanAngle(6),4)+"+-"+num2str(stdAngle(6),4), ...
    "Our 2D FMS 128 "+num2str(meanAngle(7),4)+"+-"+num2str(stdAngle(7),4), ...
    "Our FMS 2D 256 "+num2str(meanAngle(8),4)+"+-"+num2str(stdAngle(8),4), ...
    "Our Global 2D FMS 32 "+num2str(meanAngle(9),4)+"+-"+num2str(stdAngle(9),4), ...
    "Our Global 2D FMS 64 "+num2str(meanAngle(10),4)+"+-"+num2str(stdAngle(10),4), ...
    "Our Global 2D FMS 128 "+num2str(meanAngle(11),4)+"+-"+num2str(stdAngle(11),4), ...
    "Our Global FMS 2D 256 "+num2str(meanAngle(12),4)+"+-"+num2str(stdAngle(12),4), ...
    "initialGuess "+num2str(meanAngle(13),4)+"+-"+num2str(stdAngle(13),4)})
title("error angle " + whichDataset)
ylabel("error in rad")
f.Position = [100 100 900 500];
nameOfFile = "/home/tim-external/Documents/icra2023FMS/figures/"+"boxplotMeanAngles" + whichDataset;

saveas(gcf, nameOfFile, "pdf")
systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
system(systemCommand);



f = figure(5);
boxplot(timeMatrix,"Whisker",1);
ax = gca;
ax.YAxis.Scale ="log";
xticklabels({"GICP "+num2str(meanCalculationTime(1),4)+"+-"+num2str(stdCalculationTime(1),4), ...
    "Super4PCS "+num2str(meanCalculationTime(2),4)+"+-"+num2str(stdCalculationTime(2),4), ...
    "NDT D2D 2D "+num2str(meanCalculationTime(3),4)+"+-"+num2str(stdCalculationTime(3),4), ...
    "NDT P2D "+num2str(meanCalculationTime(4),4)+"+-"+num2str(stdCalculationTime(4),4), ...
    "Our 2D FMS 32 "+num2str(meanCalculationTime(5),4)+"+-"+num2str(stdCalculationTime(5),4), ...
    "Our 2D FMS 64 "+num2str(meanCalculationTime(6),4)+"+-"+num2str(stdCalculationTime(6),4), ...
    "Our 2D FMS 128 "+num2str(meanCalculationTime(7),4)+"+-"+num2str(stdCalculationTime(7),4), ...
    "Our FMS 2D 256 "+num2str(meanCalculationTime(8),4)+"+-"+num2str(stdCalculationTime(8),4), ...
    "Our Global 2D FMS 32 "+num2str(meanCalculationTime(9),4)+"+-"+num2str(stdCalculationTime(9),4), ...
    "Our Global 2D FMS 64 "+num2str(meanCalculationTime(10),4)+"+-"+num2str(stdCalculationTime(10),4), ...
    "Our Global 2D FMS 128 "+num2str(meanCalculationTime(11),4)+"+-"+num2str(stdCalculationTime(11),4), ...
    "Our Global FMS 2D 256 "+num2str(meanCalculationTime(12),4)+"+-"+num2str(stdCalculationTime(12),4), ...
    "initialGuess "+num2str(meanCalculationTime(13),4)+"+-"+num2str(stdCalculationTime(13),4)})
title("computation Time "+ whichDataset)
ylabel("computation Time in micro s")
f.Position = [100 100 900 500];

nameOfFile = "/home/tim-external/Documents/icra2023FMS/figures/"+"boxplotMeanTime" + whichDataset;

saveas(gcf, nameOfFile, "pdf")
systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
system(systemCommand);

distTMP = errorDistanceMatrix;
distTMP((distTMP(:,2)>10000),4)=0;
%distTMP = nonzeros(distTMP);

f = figure(7);
boxplot(distTMP,"Whisker",1);
ax = gca;
ax.YAxis.Scale ="log";
xticklabels({"GICP "+num2str(meanDistance(1),4)+"+-"+num2str(stdDistance(1),4), ...
    "Super4PCS "+num2str(meanDistance(2),4)+"+-"+num2str(stdDistance(2),4), ...
    "NDT D2D 2D "+num2str(meanDistance(3),4)+"+-"+num2str(stdDistance(3),4), ...
    "NDT P2D "+num2str(meanDistance(4),4)+"+-"+num2str(stdDistance(4),4), ...
    "Our 2D FMS 32 "+num2str(meanDistance(5),4)+"+-"+num2str(stdDistance(5),4), ...
    "Our 2D FMS 64 "+num2str(meanDistance(6),4)+"+-"+num2str(stdDistance(6),4), ...
    "Our 2D FMS 128 "+num2str(meanDistance(7),4)+"+-"+num2str(stdDistance(7),4), ...
    "Our FMS 2D 256 "+num2str(meanDistance(8),4)+"+-"+num2str(stdDistance(8),4), ...
    "Our Global 2D FMS 32 "+num2str(meanDistance(9),4)+"+-"+num2str(stdDistance(9),4), ...
    "Our Global 2D FMS 64 "+num2str(meanDistance(10),4)+"+-"+num2str(stdDistance(10),4), ...
    "Our Global 2D FMS 128 "+num2str(meanDistance(11),4)+"+-"+num2str(stdDistance(11),4), ...
    "Our Global FMS 2D 256 "+num2str(meanDistance(12),4)+"+-"+num2str(stdDistance(12),4), ...
    "initialGuess "+num2str(meanDistance(13),4)+"+-"+num2str(stdDistance(13),4)})
title("error l2-norm "+ whichDataset)
ylabel("error l2-norm in m")
f.Position = [100 100 900 500];
%set( gcf,'PaperSize',[29.7 21.0], 'PaperPosition',[0 0 29.7 21.0]) 
nameOfFile = "/home/tim-external/Documents/icra2023FMS/figures/"+"boxplotMeanDistance" + whichDataset;

saveas(gcf, nameOfFile, "pdf")
systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
system(systemCommand);




