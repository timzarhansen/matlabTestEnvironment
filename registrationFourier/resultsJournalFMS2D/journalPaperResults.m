clc
clear
clf
% 1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform,
% 6: Our FMS 2D 7: FMS hamming 8: FMS none
% 9: Feature0 10: Feature1  11: Feature2 12: Feature3 13: Feature4 14: Feature5
% 15: gmmRegistrationD2D 16: gmmRegistrationP2D 

% 1 whichMethod 2: voxelSize 3: initialGuess 4: errorInDistance 5: errorInRotation 6: calculationTime 7: overlap

%headmap matlab plot result

method_Of_interest = 6
voxelSize = 256;
initualGuess = 0;


journalOrdner = "/home/tim-external/dataFolder/journalPaperDatasets/";

% ordner = "highNoiseBigMotionKeller";
% ordner = "highNoiseBigMotionValentin";
% ordner = "noNoiseSmallMotionValentin";
% ordner = "noNoiseSmallMotionKeller";
% ordner = "onlyRotationNoNoiseValentin";
ordner = "onlyRotationNoNoiseKeller";


resultsExperiment = readmatrix(journalOrdner+ordner+"/"+"results.csv");


resultListOfInterest = resultsExperiment(resultsExperiment(:,1) ==method_Of_interest,:);
resultListOfInterest = resultListOfInterest(resultListOfInterest(:,2) ==voxelSize,:);
resultListOfInterest = resultListOfInterest(resultListOfInterest(:,3) ==initualGuess,:);


zeroOverlapResults = resultListOfInterest(resultListOfInterest(:,7) ==0,:);
nonZeroOverlapResults = resultListOfInterest(resultListOfInterest(:,7) ~=0,:);

initialGuessList = resultsExperiment(resultsExperiment(:,1) ==-1,:);



% remove -1 s 


initialGuessList = initialGuessList(resultListOfInterest(:,4)~=-1,:);
resultListOfInterestWithoutNANs = resultListOfInterest(resultListOfInterest(:,4)~=-1,:);

initialGuessList = initialGuessList(~isnan(resultListOfInterestWithoutNANs(:,4)),:);

resultListOfInterestWithoutNANs = resultListOfInterestWithoutNANs(~isnan(resultListOfInterestWithoutNANs(:,4)),:);

display("percent of results not NAN: " + size(resultListOfInterestWithoutNANs,1)/size(resultListOfInterest,1))
meanError = mean(resultListOfInterestWithoutNANs(:,4));
stdError = std(resultListOfInterestWithoutNANs(:,4));
display("mean: "+meanError)
display("std: " + stdError)





y = resultListOfInterestWithoutNANs(:,4);
x1 = [zeros(size(resultListOfInterestWithoutNANs(:,7))),resultListOfInterestWithoutNANs(:,7)];
x2 = [zeros(size(resultListOfInterestWithoutNANs(:,7))),initialGuessList(:,4)];

b1 = x1\y
b2 = x2\y

% b3 = resultListOfInterestWithoutNANs(:,7)\y
% b4 = initialGuessList(:,4)\y
% regression overlap
figure(1)
clf
hold on

plot(resultListOfInterestWithoutNANs(:,7),resultListOfInterestWithoutNANs(:,4),'.')
plot([0,max(resultListOfInterestWithoutNANs(:,7))],[b1(1),b1(1)+max(resultListOfInterestWithoutNANs(:,7))*b1(2)])
title("L2 overlap regression: "+b1(2))

nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/l2RegressionOverlap' + string(initualGuess)+string(voxelSize) + string(ordner) + string(method_Of_interest);
saveas(gcf,nameOfPdfFile, 'pdf' )
system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');


% set(gca, 'YScale', 'log')
% regression initial guess
figure(2)
clf
hold on 
plot(initialGuessList(:,4),resultListOfInterestWithoutNANs(:,4),'.')
plot([0,max(initialGuessList(:,4))],[b2(1),b2(1)+max(initialGuessList(:,4))*b2(2)])
title("L2 initial guess regression: "+b2(2))

% set(gca, 'YScale', 'log')
nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/l2RegressionInitialGuess' +string(initualGuess)+string(voxelSize) + string(ordner) + string(method_Of_interest);
saveas(gcf,nameOfPdfFile, 'pdf' )
system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');



figure(3)

boxplot([resultListOfInterestWithoutNANs(:,4),resultListOfInterestWithoutNANs(:,5)],'Labels',{'L2 error','angleError'})

set(gca, 'YScale', 'log')
nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/boxplot' +string(initualGuess)+string(voxelSize) + string(ordner) + string(method_Of_interest);
saveas(gcf,nameOfPdfFile, 'pdf' )
system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');

figure(4)
clf

hold on 
plot(initialGuessList(:,5),resultListOfInterestWithoutNANs(:,5),'.')
% plot([0,max(initialGuessList(:,4))],[b2(1),b2(1)+max(initialGuessList(:,4))*b2(2)])
title("angle initial guess regression: "+b2(2))



%pbaspect([1 1 1])
nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/regressionAngle' +string(initualGuess)+string(voxelSize) + string(ordner) + string(method_Of_interest);
saveas(gcf,nameOfPdfFile, 'pdf' )
system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');


