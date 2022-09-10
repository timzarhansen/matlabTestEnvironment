clc
clear

%1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 
%5: FourierMellinTransform(oneSize)(missing)
%6: Our 32 FMS 2D, 7: Our 64 FMS 2D, 8: Our 128 FMS 2D, 9: Our 256 FMS 2D


%N=19;
N=24;
% resultMatrix = readmatrix("csvFiles/comparisonAllMethodsEvenAngles.csv");

% datasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/onlyAngleValentin/";
% whichDataset = "Valentin";

datasetFolder = "/home/tim-external/dataFolder/2022FinalPaperData/onlyAngleStPere/";
whichDataset = "StPere";
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


set(groot,'defaultAxesTickLabelInterpreter','latex');  

f = figure(14)
clf
hold on 
underLimit = 45;
upperLimit = 55;
scaling = 1.0;
xticksSave = (5:5:120);%/180*pi;
scanlingToDegree = 180/pi;
generalLineWidth = 1.0;
errorbar(xticksSave,median(errorAngleGICP')*scanlingToDegree,prctile(errorAngleGICP',underLimit)*scanlingToDegree*scaling,prctile(errorAngleGICP',upperLimit)*scanlingToDegree*scaling,'o','LineWidth',generalLineWidth);
errorbar(xticksSave,median(errorAngleSUPER')*scanlingToDegree,prctile(errorAngleSUPER',underLimit)*scanlingToDegree*scaling,prctile(errorAngleGICP',upperLimit)*scanlingToDegree*scaling,'d','LineWidth',generalLineWidth);
errorbar(xticksSave,median(errorAngleNDTD2D')*scanlingToDegree,prctile(errorAngleNDTD2D',underLimit)*scanlingToDegree*scaling,prctile(errorAngleNDTD2D',upperLimit)*scanlingToDegree*scaling,'*','LineWidth',generalLineWidth);
errorbar(xticksSave,median(errorAngleNDTP2D')*scanlingToDegree,prctile(errorAngleNDTP2D',underLimit)*scanlingToDegree*scaling,prctile(errorAngleNDTP2D',upperLimit)*scanlingToDegree*scaling,'.','LineWidth',generalLineWidth);
% errorbar(xticksSave,mean(errorAngleFMSOld'),std(errorAngleFMSOld'),'.');
% errorbar(xticksSave,mean(errorAngleOURFMS32'),std(errorAngleOURFMS32'),'.');
% errorbar(xticksSave,mean(errorAngleOURFMS64'),std(errorAngleOURFMS64'),'.');
errorbar(xticksSave,median(errorAngleOURFMS128')*scanlingToDegree,prctile(errorAngleOURFMS128',underLimit)*scanlingToDegree*scaling,prctile(errorAngleOURFMS128',upperLimit)*scanlingToDegree*scaling,'x','LineWidth',generalLineWidth);
% errorbar(xticksSave,mean(errorAngleOURFMS256'),std(errorAngleOURFMS256'),'.');
%errorbar(xticksSave,mean(errorAngleOURGlobalFMS32'),scaling*(std(errorAngleOURGlobalFMS32')),'.');
%errorbar(mean(errorAngleOURGlobalFMS64'),scaling*(std(errorAngleOURGlobalFMS64')),'.');


errorbar(xticksSave,median(errorAngleOURGlobalFMS128')*scanlingToDegree,prctile(errorAngleOURGlobalFMS128',underLimit)*scanlingToDegree*scaling,prctile(errorAngleOURGlobalFMS128',upperLimit)*scanlingToDegree*scaling,'s','LineWidth',generalLineWidth);



% errorbar(xticksSave,mean(errorAngleOURGlobalFMS256')*scanlingToDegree,std(errorAngleOURGlobalFMS256')*scanlingToDegree,'s','LineWidth',generalLineWidth);
%xticklabels({})
plot(xticksSave,xticksSave,'LineWidth',1.2);




legend('GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D', 'Our 2D FMS 128', 'Our Global FMS 2D 128','error of Initial Guess','location','northwest', 'Interpreter', 'latex')
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
errorbar(xticksSave,mean(errorDistanceOURFMS128'),std(errorDistanceOURFMS128'),'x','LineWidth',generalLineWidth);
% errorbar(xticksSave,mean(errorAngleOURFMS256'),std(errorAngleOURFMS256'),'.');
%errorbar(xticksSave,mean(errorAngleOURGlobalFMS32'),scaling*(std(errorAngleOURGlobalFMS32')),'.');
%errorbar(mean(errorAngleOURGlobalFMS64'),scaling*(std(errorAngleOURGlobalFMS64')),'.');
errorbar(xticksSave,mean(errorDistanceOURGlobalFMS128'),std(errorDistanceOURGlobalFMS128'),'s','LineWidth',generalLineWidth);
%errorbar(xticksSave,mean(errorAngleOURGlobalFMS256'),std(errorAngleOURGlobalFMS256'),'.');
%xticklabels({})




legend('GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D', 'Our 2D FMS 128', 'Our Global FMS 2D 128','location','northwest', 'Interpreter', 'latex')
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










