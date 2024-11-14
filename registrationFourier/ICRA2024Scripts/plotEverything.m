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

resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/valentinTest.csv");
nameOfDataset = "Valentin128";
whereLegend = "southeast";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp3_5.csv");
% nameOfDataset = "exp3_5";
% whereLegend = "southeast";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp3_5_256.csv");
% nameOfDataset = "exp3_5_256";
% whereLegend = "southeast";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp4.csv");
% nameOfDataset = "exp4";
% whereLegend = "southeast";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/exp4_256.csv");
% nameOfDataset = "exp4_256";
% whereLegend = "east";

% resultsExperiment = readmatrix("/home/tim-external/timeMeasurements/tuhhTest.csv");
% nameOfDataset = "tuhhTest";
% whereLegend = "southeast";


% 1. time of optimization
% 2. oveall graph creation
% 3. type A
% 4. Type B
% 5. Scan Time

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
% yyaxis left
% plot(timeSinceStart,smooth(percentageOfCalcOptimization));

% ylabel("%", 'Interpreter', 'latex')

% yyaxis right
hold on
box on
grid on 
plot(timeSinceStart,smooth(scanTime));
plot(timeSinceStart,smooth(completeComputationTime))
ax=axis;
axis([ax(1:2) -1.5 ax(4)*1.001])


ylabel("seconds", 'Interpreter', 'latex')
xlabel("minutes", 'Interpreter', 'latex')

legend('scan aquisation time','overall computation time', 'Interpreter', 'latex','location',whereLegend)


pbaspect([2 1 1])
nameOfPdfFile = "/home/ws/matlab/registrationFourier/ICRA2024Scripts/pdfs/computationTimes"+nameOfDataset;
saveas(gcf,nameOfPdfFile, 'pdf' )

systemCommand = "pdfcrop " + nameOfPdfFile +".pdf "+ nameOfPdfFile+".pdf ";
system(systemCommand);


figure(2)
clf
% yyaxis left
% plot(numberOfEdges,smooth(percentageOfCalcOptimization));

% ylabel("%", 'Interpreter', 'latex')

% yyaxis right
hold on
box on 
grid on 
plot(numberOfEdges,smooth(scanTime));
plot(numberOfEdges,smooth(completeComputationTime))



ylabel("time in s", 'Interpreter', 'latex')
xlabel("number Of Edges In Graph", 'Interpreter', 'latex')

legend('scan aquisation time','overall computation time', 'Interpreter', 'latex','location',whereLegend)


pbaspect([2 1 1])
nameOfPdfFile = "/home/ws/matlab/registrationFourier/ICRA2024Scripts/pdfs/computationEdges"+nameOfDataset;
saveas(gcf,nameOfPdfFile, 'pdf' )

systemCommand = "pdfcrop " + nameOfPdfFile +".pdf "+ nameOfPdfFile+".pdf ";
system(systemCommand);


%%

% 1. time of optimization
% 2. type A
% 3. Type B
% 4. oveall graph creation
% 5. Scan Time
howManyGraphs =6;
clear barStacked
clear timeForBarPlot
for i = 1:howManyGraphs
    currentNumberOfInterest = round(size(resultsExperiment,1)/howManyGraphs);
    averageTypeA =0;
    averageTypeB =0;
    averageComputationOverload =0;
    averageOptimization =0;
    averageTime=0;
    index = 0;

    for j = (i-1)*round(size(resultsExperiment,1)/howManyGraphs)+1:(i)*round(size(resultsExperiment,1)/howManyGraphs)
        if(j>size(resultsExperiment,1))
            break
        end
        if(j>size(timeSinceStart,2))

        else
            averageTime = timeSinceStart(j);
        end
        averageTypeA =averageTypeA+resultsExperiment(j,2);
        averageTypeB =averageTypeB+resultsExperiment(j,3);
        averageComputationOverload =averageComputationOverload+resultsExperiment(j,4);
        averageOptimization =averageOptimization+resultsExperiment(j,1);
        index = index+1;
    end
    timeForBarPlot(i) = averageTime;
    barStacked(i,1)=averageComputationOverload/index;
    barStacked(i,2)=averageTypeA/index;
    barStacked(i,3)=averageTypeB/index;
    barStacked(i,4)=averageOptimization/index;
end
figure(3)



bar(barStacked,0.3,'stacked')


legend('rendering','type A','type B','iSAM2', 'Interpreter', 'latex','location','northwest')


pbaspect([2 1.0 1])

ax=axis;
axis([ax(1:2) ax(3) ax(4)*1.1])


set(gca,'XTickLabel',round(timeForBarPlot,0))

ylabel("seconds", 'Interpreter', 'latex')
xlabel("minutes", 'Interpreter', 'latex')


nameOfPdfFile = "/home/ws/matlab/registrationFourier/ICRA2024Scripts/pdfs/barGraph"+nameOfDataset;
saveas(gcf,nameOfPdfFile, 'pdf' )

systemCommand = "pdfcrop " + nameOfPdfFile +".pdf "+ nameOfPdfFile+".pdf ";
system(systemCommand);



