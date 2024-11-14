clc
clear
clf
% 1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform,
% 6: Our FMS 2D 7: FMS hamming 8: FMS none
% 9: Feature0 10: Feature1  11: Feature2 12: Feature3 13: Feature4 14: Feature5
% 15: gmmRegistrationD2D 16: gmmRegistrationP2D 

% 1 whichMethod 2: voxelSize 3: initialGuess 4: errorInDistance 5: errorInRotation 6: calculationTime 7: overlap

%headmap matlab plot result

method_Of_interest = 2
voxelSizeList = [64 ,128 ,256, 512];
initualGuess = 1;


journalOrdner = "/home/tim-external/dataFolder/journalPaperDatasets/newDatasetsCreation/";

% ordner = "highNoiseBigMotionKeller";
% ordner = "highNoiseBigMotionValentin";
% ordner = "noNoiseSmallMotionValentin";
% ordner = "noNoiseSmallMotionKeller";
% ordner = "onlyRotationNoNoiseValentin";
ordner1 = "speedTestsKeller";
ordner2 = "speedTestsValentin";

resultsExperiment1 = readmatrix(journalOrdner+ordner1+"/"+"results_seriell.csv");
resultsExperiment2 = readmatrix(journalOrdner+ordner2+"/"+"results_seriell.csv");

resultsExperiment = [resultsExperiment1 ; resultsExperiment2];

voxelSize = 256;
resultListOfInterest64 = resultsExperiment(resultsExperiment(:,2) ==64,:);
resultListOfInterest128 = resultsExperiment(resultsExperiment(:,2) ==128,:);
resultListOfInterest256 = resultsExperiment(resultsExperiment(:,2) ==voxelSize,:);
resultListOfInterest512 = resultsExperiment(resultsExperiment(:,2) ==512,:);

resultListOfInterest1 = resultListOfInterest256(resultListOfInterest256(:,1) ==1,:);% method
resultListOfInterest1 = resultListOfInterest1(resultListOfInterest1(:,3) ==1,:);%init guess



resultListOfInterest2 = resultListOfInterest256(resultListOfInterest256(:,1) ==3,:);% method
resultListOfInterest2 = resultListOfInterest2(resultListOfInterest2(:,3) ==1,:);%init guess

    resultListOfInterest3 = resultListOfInterest256(resultListOfInterest256(:,1) ==4,:);% method
resultListOfInterest3 = resultListOfInterest3(resultListOfInterest3(:,3) ==1,:);%init guess

    resultListOfInterest4 = resultListOfInterest256(resultListOfInterest256(:,1) ==5,:);% method
resultListOfInterest4 = resultListOfInterest4(resultListOfInterest4(:,3) ==1,:);%init guess



    resultListOfInterest5 = resultListOfInterest64(resultListOfInterest64(:,1) ==6,:);% method
resultListOfInterest5 = resultListOfInterest5(resultListOfInterest5(:,3) ==1,:);%init guess

    resultListOfInterest6 = resultListOfInterest64(resultListOfInterest64(:,1) ==6,:);% method
resultListOfInterest6 = resultListOfInterest6(resultListOfInterest6(:,3) ==0,:);%init guess



       resultListOfInterest7 = resultListOfInterest128(resultListOfInterest128(:,1) ==6,:);% method
resultListOfInterest7 = resultListOfInterest7(resultListOfInterest7(:,3) ==1,:);%init guess

    resultListOfInterest8 = resultListOfInterest128(resultListOfInterest128(:,1) ==6,:);% method
resultListOfInterest8 = resultListOfInterest8(resultListOfInterest8(:,3) ==0,:);%init guess


    resultListOfInterest9 = resultListOfInterest256(resultListOfInterest256(:,1) ==6,:);% method
resultListOfInterest9 = resultListOfInterest9(resultListOfInterest9(:,3) ==1,:);%init guess

    resultListOfInterest10 = resultListOfInterest256(resultListOfInterest256(:,1) ==6,:);% method
resultListOfInterest10 = resultListOfInterest10(resultListOfInterest10(:,3) ==0,:);%init guess

    resultListOfInterest11 = resultListOfInterest512(resultListOfInterest512(:,1) ==6,:);% method
resultListOfInterest11 = resultListOfInterest11(resultListOfInterest11(:,3) ==1,:);%init guess

    resultListOfInterest12 = resultListOfInterest512(resultListOfInterest512(:,1) ==6,:);% method
resultListOfInterest12 = resultListOfInterest12(resultListOfInterest12(:,3) ==0,:);%init guess




    resultListOfInterest13 = resultListOfInterest256(resultListOfInterest256(:,1) ==9,:);% method
resultListOfInterest13 = resultListOfInterest13(resultListOfInterest13(:,3) ==0,:);%init guess

    resultListOfInterest14 = resultListOfInterest256(resultListOfInterest256(:,1) ==10,:);% method
resultListOfInterest14 = resultListOfInterest14(resultListOfInterest14(:,3) ==0,:);%init guess

    resultListOfInterest15 = resultListOfInterest256(resultListOfInterest256(:,1) ==11,:);% method
resultListOfInterest15 = resultListOfInterest15(resultListOfInterest15(:,3) ==0,:);%init guess

    resultListOfInterest16 = resultListOfInterest256(resultListOfInterest256(:,1) ==12,:);% method
resultListOfInterest16 = resultListOfInterest16(resultListOfInterest16(:,3) ==0,:);%init guess

    resultListOfInterest17 = resultListOfInterest256(resultListOfInterest256(:,1) ==13,:);% method
resultListOfInterest17 = resultListOfInterest17(resultListOfInterest17(:,3) ==0,:);%init guess

    resultListOfInterest18 = resultListOfInterest256(resultListOfInterest256(:,1) ==14,:);% method
resultListOfInterest18 = resultListOfInterest18(resultListOfInterest18(:,3) ==0,:);%init guess

    resultListOfInterest19 = resultListOfInterest256(resultListOfInterest256(:,1) ==15,:);% method
resultListOfInterest19 = resultListOfInterest19(resultListOfInterest19(:,3) ==0,:);%init guess

    resultListOfInterest20 = resultListOfInterest256(resultListOfInterest256(:,1) ==16,:);% method
resultListOfInterest20 = resultListOfInterest20(resultListOfInterest20(:,3) ==0,:);%init guess



% computationTimeList = resultListOfInterest(:,6);


% figure(1)
% clf
% hold on
% 
% plot(1:size(resultListOfInterest,1),resultListOfInterest(:,6),'.')
% 
% title("ComputationTime: ")

% nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/l2RegressionOverlap' + string(initualGuess)+string(voxelSize) + string(ordner) + string(method_Of_interest);
% saveas(gcf,nameOfPdfFile, 'pdf' )
% system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');


% set(gca, 'YScale', 'log')
% regression initial guess


figure(1)
set(groot,'defaultAxesTickLabelInterpreter','latex');  
boxplot([resultListOfInterest1(:,6)/1000,resultListOfInterest2(:,6)/1000,resultListOfInterest3(:,6)/1000,resultListOfInterest4(:,6)/1000, ...
    resultListOfInterest5(:,6),resultListOfInterest6(:,6),resultListOfInterest7(:,6),resultListOfInterest8(:,6), ...
    resultListOfInterest9(:,6),resultListOfInterest10(:,6),resultListOfInterest11(:,6),resultListOfInterest12(:,6), ...
    resultListOfInterest13(:,6)/1000,resultListOfInterest14(:,6)/1000,resultListOfInterest15(:,6)/1000,resultListOfInterest16(:,6)/1000, ...
    resultListOfInterest17(:,6)/1000,resultListOfInterest18(:,6)/1000,resultListOfInterest19(:,6)/1000,resultListOfInterest20(:,6)/1000], ...
    'Labels',{'GICP','NDTD2D','NDTP2D','FourierMellin','FS2D local 64','FS2D global 64','FS2D local 128','FS2D global 128','FS2D local 256','FS2D global 256','FS2D local 512','FS2D global 512','AKAZE','KAZE','ORB','BRISK','SURF','SIFT','GMM D2D','GMM P2D'})

set(gca, 'YScale', 'log')
% title("Voxel Size: "+string(voxelSize), 'Interpreter', 'latex')

ylabel("time in ms", 'Interpreter', 'latex')

xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex'; % latex for x-axis

nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/boxplot'+string(voxelSize)+'computationSpeed';
saveas(gcf,nameOfPdfFile, 'pdf' )
% system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');
addCommandToBatchfile("batchfile.sh",'pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf &',false);





