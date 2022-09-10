clc
clear

%1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 
%5: FourierMellinTransform(oneSize)(missing)
%6: Our 32 FMS 2D, 7: Our 64 FMS 2D, 8: Our 128 FMS 2D, 9: Our 256 FMS 2D


%N=19;
N=24;
% resultMatrix = readmatrix("csvFiles/comparisonAllMethodsEvenAngles.csv");

datasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/onlyAngleValentin/";
whichDataset = "Valentin";

% datasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/onlyAngleStPere/";
% whichDataset = "StPere";
%whichError = "105";
for i =1:N
    whichError = i*5;
%      timeMatrix(i,:,:) = readmatrix("/home/tim-external/dataFolder/ValentinBunkerData/4_7_Bunker_range_30_5_OnlyAngle"+ whichError +"/calculationTime1.csv");
%      errorAngleMatrix(i,:,:) = readmatrix("/home/tim-external/dataFolder/ValentinBunkerData/4_7_Bunker_range_30_5_OnlyAngle"+ whichError +"/errorAngle1.csv");
%      errorDistanceMatrix(i,:,:) = readmatrix("/home/tim-external/dataFolder/ValentinBunkerData/4_7_Bunker_range_30_5_OnlyAngle"+ whichError +"/errorDistance1.csv");
     
    timeMatrix(i,:,:) = readmatrix(datasetFolder+ whichError +"degree/calculationTime1.csv");
    errorAngleMatrix(i,:,:) = readmatrix(datasetFolder+ whichError +"degree/errorAngle1.csv");
    errorDistanceMatrix(i,:,:) = readmatrix(datasetFolder+ whichError +"degree/errorDistance1.csv");

end






for i = 1:size(errorAngleMatrix,1)
    errorAngleGICP(i,:) = errorAngleMatrix(i,:,1);
    errorAngleSUPER(i,:) = errorAngleMatrix(i,:,2);
    errorAngleNDTD2D(i,:) = errorAngleMatrix(i,:,3);
    errorAngleNDTP2D(i,:) = errorAngleMatrix(i,:,4);
    errorAngleOURFMS32(i,:) = errorAngleMatrix(i,:,5);
    errorAngleOURFMS64(i,:) = errorAngleMatrix(i,:,6);
    errorAngleOURFMS128(i,:) = errorAngleMatrix(i,:,7);
    errorAngleOURFMS256(i,:) = errorAngleMatrix(i,:,8);
    errorAngleOURGlobalFMS32(i,:) = errorAngleMatrix(i,:,9);
    errorAngleOURGlobalFMS64(i,:) = errorAngleMatrix(i,:,10);
    errorAngleOURGlobalFMS128(i,:) = errorAngleMatrix(i,:,11);
    errorAngleOURGlobalFMS256(i,:) = errorAngleMatrix(i,:,12);





    errorDistanceGICP(i,:) = errorDistanceMatrix(i,:,1);
    errorDistanceSUPER(i,:) = errorDistanceMatrix(i,:,2);
    errorDistanceNDTD2D(i,:) = errorDistanceMatrix(i,:,3);
    errorDistanceNDTP2D(i,:) = errorDistanceMatrix(i,:,4);
    errorDistanceOURFMS32(i,:) = errorDistanceMatrix(i,:,5);
    errorDistanceOURFMS64(i,:) = errorDistanceMatrix(i,:,6);
    errorDistanceOURFMS128(i,:) = errorDistanceMatrix(i,:,7);
    errorDistanceOURFMS256(i,:) = errorDistanceMatrix(i,:,8);
    errorDistanceOURGlobalFMS32(i,:) = errorDistanceMatrix(i,:,9);
    errorDistanceOURGlobalFMS64(i,:) = errorDistanceMatrix(i,:,10);
    errorDistanceOURGlobalFMS128(i,:) = errorDistanceMatrix(i,:,11);
    errorDistanceOURGlobalFMS256(i,:) = errorDistanceMatrix(i,:,12);


    timeGICP(i,:) = timeMatrix(i,:,1);
    timeSUPER(i,:) = timeMatrix(i,:,2);
    timeNDTD2D(i,:) = timeMatrix(i,:,3);
    timeNDTP2D(i,:) = timeMatrix(i,:,4);
    timeOURFMS32(i,:) = timeMatrix(i,:,5);
    timeOURFMS64(i,:) = timeMatrix(i,:,6);
    timeOURFMS128(i,:) = timeMatrix(i,:,7);
    timeOURFMS256(i,:) = timeMatrix(i,:,8);
    timeOURGlobalFMS32(i,:) = timeMatrix(i,:,9);
    timeOURGlobalFMS64(i,:) = timeMatrix(i,:,10);
    timeOURGlobalFMS128(i,:) = timeMatrix(i,:,11);
    timeOURGlobalFMS256(i,:) = timeMatrix(i,:,12);

end


%%
% f = figure(1);
% boxplot(errorAngleGICP','Whisker',1);
% f = figure(2);
% boxplot(errorAngleSUPER','Whisker',1);
% f = figure(3);
% boxplot(errorAngleNDTD2D','Whisker',1);
% f = figure(4);
% boxplot(errorAngleNDTP2D','Whisker',1);
% f = figure(6);
% boxplot(errorAngleOURFMS32','Whisker',1);
% f = figure(7);
% boxplot(errorAngleOURFMS64','Whisker',1);
% f = figure(8);
% boxplot(errorAngleOURFMS128','Whisker',1);
% f = figure(9);
% boxplot(errorAngleOURFMS256','Whisker',1);
% title('local 256')
% f = figure(10);
% boxplot(errorAngleOURGlobalFMS32','Whisker',1);
% title('global 32')
% f = figure(11);
% boxplot(errorAngleOURGlobalFMS64','Whisker',1);
% title('global 64')
% f = figure(12);
% boxplot(errorAngleOURGlobalFMS128','Whisker',1);
% title('global 128')
% f = figure(13);
% boxplot(errorAngleOURGlobalFMS256','Whisker',1);
% title('global 256')

%%

% f = figure(1);
% plot(mean(errorAngleGICP'),'.');
% f = figure(2);
% plot(mean(errorAngleSUPER'),'.');
% f = figure(3);
% plot(mean(errorAngleNDTD2D'),'.');
% f = figure(4);
% plot(mean(errorAngleNDTP2D'),'.');
% f = figure(6);
% plot(mean(errorAngleOURFMS32'),'.');
% f = figure(7);
% plot(mean(errorAngleOURFMS64'),'.');
% f = figure(8);
% plot(mean(errorAngleOURFMS128'),'.');
% f = figure(9);
% plot(mean(errorAngleOURFMS256'),'.');
% f = figure(10);
% plot(mean(errorAngleOURGlobalFMS32'),'.');
% title('global 32')
% f = figure(11);
% plot(mean(errorAngleOURGlobalFMS64'),'.');
% title('global 64')
% f = figure(12);
% plot(mean(errorAngleOURGlobalFMS128'),'.');
% title('global 128')
% f = figure(13);
% plot(mean(errorAngleOURGlobalFMS256'),'.');
% title('global 256')


%%


set(groot,'defaultAxesTickLabelInterpreter','latex');  

f = figure(14)
clf
hold on 
scaling = 0.2;
xticksSave = (5:5:120);%/180*pi;
scanlingToDegree = 180/pi;
generalLineWidth = 1.0;

errorbar(xticksSave,mean(errorAngleGICP')*scanlingToDegree,scaling*(std(errorAngleGICP'))*scanlingToDegree,'o','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorAngleSUPER')*scanlingToDegree,scaling*(std(errorAngleSUPER'))*scanlingToDegree,'d','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorAngleNDTD2D')*scanlingToDegree,scaling*(std(errorAngleNDTD2D'))*scanlingToDegree,'*','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorAngleNDTP2D')*scanlingToDegree,scaling*(std(errorAngleNDTP2D'))*scanlingToDegree,'.','LineWidth',generalLineWidth);
% errorbar(xticksSave,mean(errorAngleFMSOld'),std(errorAngleFMSOld'),'.');
% errorbar(xticksSave,mean(errorAngleOURFMS32'),std(errorAngleOURFMS32'),'.');
% errorbar(xticksSave,mean(errorAngleOURFMS64'),std(errorAngleOURFMS64'),'.');
% errorbar(xticksSave,mean(errorAngleOURFMS128')*scanlingToDegree,std(errorAngleOURFMS128')*scanlingToDegree,'x','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorAngleOURFMS256')*scanlingToDegree,std(errorAngleOURFMS256')*scanlingToDegree,'x','LineWidth',generalLineWidth);
%errorbar(xticksSave,mean(errorAngleOURGlobalFMS32'),scaling*(std(errorAngleOURGlobalFMS32')),'.');
%errorbar(mean(errorAngleOURGlobalFMS64'),scaling*(std(errorAngleOURGlobalFMS64')),'.');
% errorbar(xticksSave,mean(errorAngleOURGlobalFMS128')*scanlingToDegree,std(errorAngleOURGlobalFMS128')*scanlingToDegree,'s','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorAngleOURGlobalFMS256')*scanlingToDegree,std(errorAngleOURGlobalFMS256')*scanlingToDegree,'s','LineWidth',generalLineWidth);
%xticklabels({})
plot(xticksSave,xticksSave,'LineWidth',1.2);




legend('GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D', 'Our 2D FMS 256', 'Our Global FMS 2D 256','error of Initial Guess','location','northwest', 'Interpreter', 'latex')
ylabel("absolute angle error in degree", 'Interpreter', 'latex')
xlabel("scan rotated in degree", 'Interpreter', 'latex')

box on
grid on
f.Position = [100 100 900 500];
ax = gca;
ax.XLim = [0 125];

nameOfFile = "/home/tim-external/Documents/icra2023FMS/figures/"+"error_angle_Rotating_scans" + whichDataset;
saveas(gcf,nameOfFile , 'pdf');
systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
system(systemCommand);

%%
set(groot,'defaultAxesTickLabelInterpreter','latex');  

f = figure(15)
clf
hold on 
scaling = 0.2;
xticksSave = (5:5:120);
generalLineWidth = 1.0;
%plot(xticksSave,xticksSave,'LineWidth',2);







errorbar(xticksSave,mean(errorDistanceGICP'),scaling*(std(errorDistanceGICP')),'o','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorDistanceSUPER'),scaling*(std(errorDistanceSUPER')),'d','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorDistanceNDTD2D'),scaling*(std(errorDistanceNDTD2D')),'*','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorDistanceNDTP2D'),scaling*(std(errorDistanceNDTP2D')),'.','LineWidth',generalLineWidth);
% errorbar(xticksSave,mean(errorAngleFMSOld'),std(errorAngleFMSOld'),'.');
% errorbar(xticksSave,mean(errorAngleOURFMS32'),std(errorAngleOURFMS32'),'.');
% errorbar(xticksSave,mean(errorAngleOURFMS64'),std(errorAngleOURFMS64'),'.');
% errorbar(xticksSave,mean(errorDistanceOURFMS128'),std(errorDistanceOURFMS128'),'x','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorDistanceOURFMS256'),std(errorDistanceOURFMS256'),'x','LineWidth',generalLineWidth);

%errorbar(xticksSave,mean(errorAngleOURGlobalFMS32'),scaling*(std(errorAngleOURGlobalFMS32')),'.');
%errorbar(mean(errorAngleOURGlobalFMS64'),scaling*(std(errorAngleOURGlobalFMS64')),'.');
% errorbar(xticksSave,mean(errorDistanceOURGlobalFMS128'),std(errorDistanceOURGlobalFMS128'),'s','LineWidth',generalLineWidth);
errorbar(xticksSave,mean(errorDistanceOURGlobalFMS256'),std(errorDistanceOURGlobalFMS256'),'s','LineWidth',generalLineWidth);
%xticklabels({})




legend('GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D', 'Our 2D FMS 256', 'Our Global FMS 2D 256','location','northwest', 'Interpreter', 'latex')
ylabel("absolute distance error in m", 'Interpreter', 'latex')
xlabel("scan rotated by degree", 'Interpreter', 'latex')

ax = gca;
ax.XLim = [0 125];

f.Position = [100 100 900 500];
box on
grid on
nameOfFile = "/home/tim-external/Documents/icra2023FMS/figures/"+"error_distance_Rotating_scans" + whichDataset;
saveas(gcf,nameOfFile , 'pdf');
systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
system(systemCommand);










