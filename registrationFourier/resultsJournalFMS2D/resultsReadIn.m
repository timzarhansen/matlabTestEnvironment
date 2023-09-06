clc
clear
% 1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform, 6: Our FMS 2D 7: initial guess
% 1 whichMethod 2: voxelSize 3: initialGuess 4: errorInDistance 5: errorInRotation 6: calculationTime
resultsExperiment = readmatrix("results.csv");
a = (1:22)'; 
b = (1:22)'; 
city = {'GICP';'GICP';'GICP';
    'SUPER4PCS';'SUPER4PCS';'SUPER4PCS';
    'NDT D2D 2D';'NDT D2D 2D';'NDT D2D 2D';
    'NDT P2D';'NDT P2D';'NDT P2D';
    'FourierMellinTransform';'FourierMellinTransform';'FourierMellinTransform';
    'Our FMS 2D';'Our FMS 2D';'Our FMS 2D';
    'Our FMS 2D Global';'Our FMS 2D Global';'Our FMS 2D Global';
    'initial guess'};
resultTable = table(city,a,a,a,a,a,a,a,'VariableNames',{'registrationMethod','voxelSize','meanL2','stdL2','meanAngle','stdAngle','meanComputationTime','stdComputationTime'});



% initial guess
resultsInitialGuess = resultsExperiment(resultsExperiment(:,1) ==-1,:);
resultTable(22,2) = {0};
resultTable(22,3)={mean(abs(resultsInitialGuess(:,4)))};
resultTable(22,4)={std(resultsInitialGuess(:,4))};
resultTable(22,5)={mean(abs(resultsInitialGuess(:,5)))};
resultTable(22,6)={std(resultsInitialGuess(:,5))};
resultTable(22,7)={mean(abs(resultsInitialGuess(:,6)))};
resultTable(22,8)={std(resultsInitialGuess(:,6))};



% GICP
resultsGICP = resultsExperiment(resultsExperiment(:,1) == 1,:);
resultsGICP128 = resultsGICP(resultsGICP(:,2) == 128,:);
resultTable(1,2) = {128};
resultTable(1,3)={mean(abs(resultsGICP128(:,4)))};
resultTable(1,4)={std(abs(resultsGICP128(:,4)))};
resultTable(1,5)={mean(abs(resultsGICP128(:,5)))};
resultTable(1,6)={std(abs(resultsGICP128(:,5)))};
resultTable(1,7)={mean(abs(resultsGICP128(:,6)))};
resultTable(1,8)={std(abs(resultsGICP128(:,6)))};


resultsGICP256 = resultsGICP(resultsGICP(:,2) == 256,:);
resultTable(2,2) = {256};
resultTable(2,3)={mean(abs(resultsGICP256(:,4)))};
resultTable(2,4)={std(resultsGICP256(:,4))};
resultTable(2,5)={mean(abs(resultsGICP256(:,5)))};
resultTable(2,6)={std(resultsGICP256(:,5))};
resultTable(2,7)={mean(abs(resultsGICP256(:,6)))};
resultTable(2,8)={std(resultsGICP256(:,6))};


resultsGICP512 = resultsGICP(resultsGICP(:,2) == 512,:);
resultTable(3,2) = {512};
resultTable(3,3)={mean(abs(resultsGICP512(:,4)))};
resultTable(3,4)={std(resultsGICP512(:,4))};
resultTable(3,5)={mean(abs(resultsGICP512(:,5)))};
resultTable(3,6)={std(resultsGICP512(:,5))};
resultTable(3,7)={mean(abs(resultsGICP512(:,6)))};
resultTable(3,8)={std(resultsGICP512(:,6))};

% SUPER 4PCS
resultsSUPER4PCS = resultsExperiment(resultsExperiment(:,1) == 2,:);
resultsSUPER4PCS128 = resultsSUPER4PCS(resultsSUPER4PCS(:,2) == 128,:);
resultTable(4,2) = {128};
resultTable(4,3)={mean(abs(resultsSUPER4PCS128(:,4)))};
resultTable(4,4)={std(resultsSUPER4PCS128(:,4))};
resultTable(4,5)={mean(abs(resultsSUPER4PCS128(:,5)))};
resultTable(4,6)={std(resultsSUPER4PCS128(:,5))};
resultTable(4,7)={mean(abs(resultsSUPER4PCS128(:,6)))};
resultTable(4,8)={std(resultsSUPER4PCS128(:,6))};



resultsSUPER4PCS256 = resultsSUPER4PCS(resultsSUPER4PCS(:,2) == 256,:);
resultTable(5,2) = {256};
resultTable(5,3)={mean(abs(resultsSUPER4PCS256(:,4)))};
resultTable(5,4)={std(resultsSUPER4PCS256(:,4))};
resultTable(5,5)={mean(abs(resultsSUPER4PCS256(:,5)))};
resultTable(5,6)={std(resultsSUPER4PCS256(:,5))};
resultTable(5,7)={mean(abs(resultsSUPER4PCS256(:,6)))};
resultTable(5,8)={std(resultsSUPER4PCS256(:,6))};

resultsSUPER4PCS512 = resultsSUPER4PCS(resultsSUPER4PCS(:,2) == 512,:);
resultTable(6,2) = {512};
resultTable(6,3)={mean(abs(resultsSUPER4PCS512(:,4)))};
resultTable(6,4)={std(resultsSUPER4PCS512(:,4))};
resultTable(6,5)={mean(abs(resultsSUPER4PCS512(:,5)))};
resultTable(6,6)={std(resultsSUPER4PCS512(:,5))};
resultTable(6,7)={mean(abs(resultsSUPER4PCS512(:,6)))};
resultTable(6,8)={std(resultsSUPER4PCS512(:,6))};

% NDTD2D2D
resultsNDTD2D2D = resultsExperiment(resultsExperiment(:,1) == 3,:);
resultsNDTD2D2D128 = resultsNDTD2D2D(resultsNDTD2D2D(:,2) == 128,:);
resultTable(7,2) = {128};
resultTable(7,3)={mean(abs(resultsNDTD2D2D128(:,4)))};
resultTable(7,4)={std(resultsNDTD2D2D128(:,4))};
resultTable(7,5)={mean(abs(resultsNDTD2D2D128(:,5)))};
resultTable(7,6)={std(resultsNDTD2D2D128(:,5))};
resultTable(7,7)={mean(abs(resultsNDTD2D2D128(:,6)))};
resultTable(7,8)={std(resultsNDTD2D2D128(:,6))};


resultsNDTD2D2D256 = resultsNDTD2D2D(resultsNDTD2D2D(:,2) == 256,:);
resultTable(8,2) = {256};
resultTable(8,3)={mean(abs(resultsNDTD2D2D256(:,4)))};
resultTable(8,4)={std(resultsNDTD2D2D256(:,4))};
resultTable(8,5)={mean(abs(resultsNDTD2D2D256(:,5)))};
resultTable(8,6)={std(resultsNDTD2D2D256(:,5))};
resultTable(8,7)={mean(abs(resultsNDTD2D2D256(:,6)))};
resultTable(8,8)={std(resultsNDTD2D2D256(:,6))};

resultsNDTD2D2D512 = resultsNDTD2D2D(resultsNDTD2D2D(:,2) == 512,:);
resultTable(9,2) = {512};
resultTable(9,3)={mean(abs(resultsNDTD2D2D512(:,4)))};
resultTable(9,4)={std(resultsNDTD2D2D512(:,4))};
resultTable(9,5)={mean(abs(resultsNDTD2D2D512(:,5)))};
resultTable(9,6)={std(resultsNDTD2D2D512(:,5))};
resultTable(9,7)={mean(abs(resultsNDTD2D2D512(:,6)))};
resultTable(9,8)={std(resultsNDTD2D2D512(:,6))};

% NDT P2D
resultsNDTP2D = resultsExperiment(resultsExperiment(:,1) == 5,:);
resultsNDTP2D128 = resultsNDTP2D(resultsNDTP2D(:,2) == 128,:);
resultTable(10,2) = {128};
resultTable(10,3)={mean(abs(resultsNDTP2D128(:,4)))};
resultTable(10,4)={std(resultsNDTP2D128(:,4))};
resultTable(10,5)={mean(abs(resultsNDTP2D128(:,5)))};
resultTable(10,6)={std(resultsNDTP2D128(:,5))};
resultTable(10,7)={mean(abs(resultsNDTP2D128(:,6)))};
resultTable(10,8)={std(resultsNDTP2D128(:,6))};

resultsNDTP2D256 = resultsNDTP2D(resultsNDTP2D(:,2) == 256,:);
resultTable(11,2) = {256};
resultTable(11,3)={mean(abs(resultsNDTP2D256(:,4)))};
resultTable(11,4)={std(resultsNDTP2D256(:,4))};
resultTable(11,5)={mean(abs(resultsNDTP2D256(:,5)))};
resultTable(11,6)={std(resultsNDTP2D256(:,5))};
resultTable(11,7)={mean(abs(resultsNDTP2D256(:,6)))};
resultTable(11,8)={std(resultsNDTP2D256(:,6))};

resultsNDTP2D512 = resultsNDTP2D(resultsNDTP2D(:,2) == 512,:);
resultTable(12,2) = {512};
resultTable(12,3)={mean(abs(resultsNDTP2D512(:,4)))};
resultTable(12,4)={std(resultsNDTP2D512(:,4))};
resultTable(12,5)={mean(abs(resultsNDTP2D512(:,5)))};
resultTable(12,6)={std(resultsNDTP2D512(:,5))};
resultTable(12,7)={mean(abs(resultsNDTP2D512(:,6)))};
resultTable(12,8)={std(resultsNDTP2D512(:,6))};

%fourier Mellin
resultsfourierMellin = resultsExperiment(resultsExperiment(:,1) == 5,:);
resultsfourierMellin128 = resultsfourierMellin(resultsfourierMellin(:,2) == 128,:);
resultTable(13,2) = {128};
resultTable(13,3)={mean(abs(resultsfourierMellin128(:,4)))};
resultTable(13,4)={std(resultsfourierMellin128(:,4))};
resultTable(13,5)={mean(abs(resultsfourierMellin128(:,5)))};
resultTable(13,6)={std(resultsfourierMellin128(:,5))};
resultTable(13,7)={mean(abs(resultsfourierMellin128(:,6)))};
resultTable(13,8)={std(resultsfourierMellin128(:,6))};

resultsfourierMellin256 = resultsfourierMellin(resultsfourierMellin(:,2) == 256,:);
resultTable(14,2) = {256};
resultTable(14,3)={mean(abs(resultsfourierMellin256(:,4)))};
resultTable(14,4)={std(resultsfourierMellin256(:,4))};
resultTable(14,5)={mean(abs(resultsfourierMellin256(:,5)))};
resultTable(14,6)={std(resultsfourierMellin256(:,5))};
resultTable(14,7)={mean(abs(resultsfourierMellin256(:,6)))};
resultTable(14,8)={std(resultsfourierMellin256(:,6))};

resultsfourierMellin512 = resultsfourierMellin(resultsfourierMellin(:,2) == 512,:);
resultTable(15,2) = {512};
resultTable(15,3)={mean(abs(resultsfourierMellin512(:,4)))};
resultTable(15,4)={std(resultsfourierMellin512(:,4))};
resultTable(15,5)={mean(abs(resultsfourierMellin512(:,5)))};
resultTable(15,6)={std(resultsfourierMellin512(:,5))};
resultTable(15,7)={mean(abs(resultsfourierMellin512(:,6)))};
resultTable(15,8)={std(resultsfourierMellin512(:,6))};

% Our FMS 2D
resultsOurFMS2D = resultsExperiment(resultsExperiment(:,1) == 6,:);
resultsOurFMS2D = resultsOurFMS2D(resultsOurFMS2D(:,3) == 1,:);

resultsOurFMS2DIG128 = resultsOurFMS2D(resultsOurFMS2D(:,2) == 128,:);
resultTable(16,2) = {128};
resultTable(16,3)={mean(abs(resultsOurFMS2DIG128(:,4)))};
resultTable(16,4)={std(resultsOurFMS2DIG128(:,4))};
resultTable(16,5)={mean(abs(resultsOurFMS2DIG128(:,5)))};
resultTable(16,6)={std(resultsOurFMS2DIG128(:,5))};
resultTable(16,7)={mean(abs(resultsOurFMS2DIG128(:,6)))};
resultTable(16,8)={std(resultsOurFMS2DIG128(:,6))};

resultsOurFMS2DIG256 = resultsOurFMS2D(resultsOurFMS2D(:,2) == 256,:);
resultTable(17,2) = {256};
resultTable(17,3)={mean(abs(resultsOurFMS2DIG256(:,4)))};
resultTable(17,4)={std(resultsOurFMS2DIG256(:,4))};
resultTable(17,5)={mean(abs(resultsOurFMS2DIG256(:,5)))};
resultTable(17,6)={std(resultsOurFMS2DIG256(:,5))};
resultTable(17,7)={mean(abs(resultsOurFMS2DIG256(:,6)))};
resultTable(17,8)={std(resultsOurFMS2DIG256(:,6))};

resultsOurFMS2DIG512 = resultsOurFMS2D(resultsOurFMS2D(:,2) == 512,:);
resultTable(18,2) = {512};
resultTable(18,3)={mean(abs(resultsOurFMS2DIG512(:,4)))};
resultTable(18,4)={std(resultsOurFMS2DIG512(:,4))};
resultTable(18,5)={mean(abs(resultsOurFMS2DIG512(:,5)))};
resultTable(18,6)={std(resultsOurFMS2DIG512(:,5))};
resultTable(18,7)={mean(abs(resultsOurFMS2DIG512(:,6)))};
resultTable(18,8)={std(resultsOurFMS2DIG512(:,6))};

% Our FMS 2D global
resultsOurFMS2D = resultsExperiment(resultsExperiment(:,1) == 6,:);
resultsOurFMS2D = resultsOurFMS2D(resultsOurFMS2D(:,3) == 0,:);

resultsOurFMS2DGG128 = resultsOurFMS2D(resultsOurFMS2D(:,2) == 128,:);
resultTable(19,2) = {128};
resultTable(19,3)={mean(abs(resultsOurFMS2DGG128(:,4)))};
resultTable(19,4)={std(resultsOurFMS2DGG128(:,4))};
resultTable(19,5)={mean(abs(resultsOurFMS2DGG128(:,5)))};
resultTable(19,6)={std(resultsOurFMS2DGG128(:,5))};
resultTable(19,7)={mean(abs(resultsOurFMS2DGG128(:,6)))};
resultTable(19,8)={std(resultsOurFMS2DGG128(:,6))};


resultsOurFMS2DGG256 = resultsOurFMS2D(resultsOurFMS2D(:,2) == 256,:);
resultTable(20,2) = {256};
resultTable(20,3)={mean(abs(resultsOurFMS2DGG256(:,4)))};
resultTable(20,4)={std(resultsOurFMS2DGG256(:,4))};
resultTable(20,5)={mean(abs(resultsOurFMS2DGG256(:,5)))};
resultTable(20,6)={std(resultsOurFMS2DGG256(:,5))};
resultTable(20,7)={mean(abs(resultsOurFMS2DGG256(:,6)))};
resultTable(20,8)={std(resultsOurFMS2DGG256(:,6))};

resultsOurFMS2DGG512 = resultsOurFMS2D(resultsOurFMS2D(:,2) == 512,:);
resultTable(21,2) = {512};
resultTable(21,3)={mean(abs(resultsOurFMS2DGG512(:,4)))};
resultTable(21,4)={std(resultsOurFMS2DGG512(:,4))};
resultTable(21,5)={mean(abs(resultsOurFMS2DGG512(:,5)))};
resultTable(21,6)={std(resultsOurFMS2DGG512(:,5))};
resultTable(21,7)={mean(abs(resultsOurFMS2DGG512(:,6)))};
resultTable(21,8)={std(resultsOurFMS2DGG512(:,6))};


%%
figure(1)
clf
boxplot([resultsGICP128(:,4),resultsGICP256(:,4),resultsGICP512(:,4), ...
    resultsNDTD2D2D128(:,4),resultsNDTD2D2D256(:,4),resultsNDTD2D2D512(:,4), ...
    resultsNDTP2D128(:,4),resultsNDTP2D256(:,4),resultsNDTP2D512(:,4), ...
    resultsfourierMellin128(:,4),resultsfourierMellin256(:,4),resultsfourierMellin512(:,4), ...
    resultsOurFMS2DIG128(:,4),resultsOurFMS2DIG256(:,4),resultsOurFMS2DIG512(:,4), ...
    resultsOurFMS2DGG128(:,4),resultsOurFMS2DGG256(:,4),resultsOurFMS2DGG512(:,4), ...
    resultsInitialGuess(:,4)],'Labels',{'GICP 128';'GICP 256';'GICP 512'; ...
    'NDT D2D 2D 128';'NDT D2D 2D 256';'NDT D2D 2D 512'; ...
    'NDT P2D 128';'NDT P2D 256';'NDT P2D 512'; ...
    'FourierMellinTransform 128';'FourierMellinTransform 256';'FourierMellinTransform 512'; ...
    'Our FMS 2D 128';'Our FMS 2D 256';'Our FMS 2D 512'; ...
    'Our FMS 2D Global 128';'Our FMS 2D Global 256';'Our FMS 2D Global 512'; ...
    'initial guess'})

ax = gca;
ax.YAxis.Scale ="log";
title("L2 norm Error")


figure(2)
clf
boxplot([resultsGICP128(:,5),resultsGICP256(:,5),resultsGICP512(:,5), ...
    resultsNDTD2D2D128(:,5),resultsNDTD2D2D256(:,5),resultsNDTD2D2D512(:,5), ...
    resultsNDTP2D128(:,5),resultsNDTP2D256(:,5),resultsNDTP2D512(:,5), ...
    resultsfourierMellin128(:,5),resultsfourierMellin256(:,5),resultsfourierMellin512(:,5), ...
    resultsOurFMS2DIG128(:,5),resultsOurFMS2DIG256(:,5),resultsOurFMS2DIG512(:,5), ...
    resultsOurFMS2DGG128(:,5),resultsOurFMS2DGG256(:,5),resultsOurFMS2DGG512(:,5), ...
    resultsInitialGuess(:,5)],'Labels',{'GICP 128';'GICP 256';'GICP 512'; ...
    'NDT D2D 2D 128';'NDT D2D 2D 256';'NDT D2D 2D 512'; ...
    'NDT P2D 128';'NDT P2D 256';'NDT P2D 512'; ...
    'FourierMellinTransform 128';'FourierMellinTransform 256';'FourierMellinTransform 512'; ...
    'Our FMS 2D 128';'Our FMS 2D 256';'Our FMS 2D 512'; ...
    'Our FMS 2D Global 128';'Our FMS 2D Global 256';'Our FMS 2D Global 512'; ...
    'initial guess'})

ax = gca;
ax.YAxis.Scale ="log";
title("abs angle error")


figure(3)
clf
boxplot([resultsGICP128(:,6),resultsGICP256(:,6),resultsGICP512(:,6), ...
    resultsNDTD2D2D128(:,6),resultsNDTD2D2D256(:,6),resultsNDTD2D2D512(:,6), ...
    resultsNDTP2D128(:,6),resultsNDTP2D256(:,6),resultsNDTP2D512(:,6), ...
    resultsfourierMellin128(:,6),resultsfourierMellin256(:,6),resultsfourierMellin512(:,6), ...
    resultsOurFMS2DIG128(:,6),resultsOurFMS2DIG256(:,6),resultsOurFMS2DIG512(:,6), ...
    resultsOurFMS2DGG128(:,6),resultsOurFMS2DGG256(:,6),resultsOurFMS2DGG512(:,6), ...
    resultsInitialGuess(:,6)],'Labels',{'GICP 128';'GICP 256';'GICP 512'; ...
    'NDT D2D 2D 128';'NDT D2D 2D 256';'NDT D2D 2D 512'; ...
    'NDT P2D 128';'NDT P2D 256';'NDT P2D 512'; ...
    'FourierMellinTransform 128';'FourierMellinTransform 256';'FourierMellinTransform 512'; ...
    'Our FMS 2D 128';'Our FMS 2D 256';'Our FMS 2D 512'; ...
    'Our FMS 2D Global 128';'Our FMS 2D Global 256';'Our FMS 2D Global 512'; ...
    'initial guess'})

ax = gca;
ax.YAxis.Scale ="log";
title("computational time")


%%

arrayToWorkOn = resultsGICP256;

plot(arrayToWorkOn(:,7),arrayToWorkOn(:,4),'.')
% ax = gca;
% ax.YAxis.Scale ="log";

c = polyfit(arrayToWorkOn(:,7),arrayToWorkOn(:,4),1)
y_est = polyval(c,arrayToWorkOn(:,7));
% Add trend line to plot
hold on
plot(arrayToWorkOn(:,7),y_est,'r--','LineWidth',2)
hold off


