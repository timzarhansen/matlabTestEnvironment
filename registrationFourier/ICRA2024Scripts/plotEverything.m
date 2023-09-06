clc
clear
clf

set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
factor = 1;





% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/valentinTest256.csv");
% nameOfDataset = "Valentin256";
% whereLegend = "southeast";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/valentinTest.csv");
% nameOfDataset = "Valentin128";
% whereLegend = "east";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp3_5.csv");
% nameOfDataset = "exp3_5";
% whereLegend = "east";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp3_5_256.csv");
% nameOfDataset = "exp3_5_256";
% whereLegend = "southeast";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp4.csv");
% nameOfDataset = "exp4";
% whereLegend = "southeast";

resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp4_256.csv");
nameOfDataset = "exp4_256";
whereLegend = "east";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/tuhhTest.csv");
% nameOfDataset = "tuhhTest";
% whereLegend = "southeast";




timeSinceStart(1) = 0;
for i = 2:size(resultsExperiment,1)
    
    percentageOfCalcOptimization(i-1) = resultsExperiment(i,1)/(resultsExperiment(i,1)+resultsExperiment(i,2)+resultsExperiment(i,3)+resultsExperiment(i,4));
    scanTime(i-1) = resultsExperiment(i,5);
    completeComputationTime(i-1) = resultsExperiment(i,1)+resultsExperiment(i,2)+resultsExperiment(i,3)+resultsExperiment(i,4);
    numberOfNodes(i-1) = resultsExperiment(i,6);
    numberOfEdges(i-1) = resultsExperiment(i,7);
    timeSinceStart(i) = timeSinceStart(i-1)+scanTime(i-1)/60.0;

end
timeSinceStart = timeSinceStart(2:end);

figure(1)
clf
yyaxis left
plot(timeSinceStart,smooth(percentageOfCalcOptimization));

ylabel("%", 'Interpreter', 'latex')

yyaxis right
hold on
plot(timeSinceStart,smooth(scanTime));
plot(timeSinceStart,smooth(completeComputationTime))



ylabel("time in s", 'Interpreter', 'latex')
xlabel("SLAM running in minutes", 'Interpreter', 'latex')

legend(' iSam2 optimization','scan aquisation time','overall computation time', 'Interpreter', 'latex','location',whereLegend)


pbaspect([2 1 1])
nameOfPdfFile = "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/ICRA2024Scripts/pdfs/computationTimes"+nameOfDataset;
saveas(gcf,nameOfPdfFile, 'pdf' )

systemCommand = "pdfcrop " + nameOfPdfFile +".pdf "+ nameOfPdfFile+".pdf ";
system(systemCommand);


figure(2)
clf
yyaxis left
plot(numberOfEdges,smooth(percentageOfCalcOptimization));

ylabel("%", 'Interpreter', 'latex')

yyaxis right
hold on
plot(numberOfEdges,smooth(scanTime));
plot(numberOfEdges,smooth(completeComputationTime))



ylabel("time in s", 'Interpreter', 'latex')
xlabel("number Of Edges In Graph", 'Interpreter', 'latex')

legend(' iSam2 optimization','scan aquisation time','overall computation time', 'Interpreter', 'latex','location',whereLegend)


pbaspect([2 1 1])
nameOfPdfFile = "/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/ICRA2024Scripts/pdfs/computationEdges"+nameOfDataset;
saveas(gcf,nameOfPdfFile, 'pdf' )

systemCommand = "pdfcrop " + nameOfPdfFile +".pdf "+ nameOfPdfFile+".pdf ";
system(systemCommand);