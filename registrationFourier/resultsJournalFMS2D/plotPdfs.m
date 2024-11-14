function [] = plotPdfs(method_Of_interest,voxelSize,initualGuess,ordner,usePoints,sizeOfGrid,numberOfCombinedDatasets,nameOfBatchFile,ymax,plotNumbers,l2NormMax)
%     clc
%     clear
%     clf
    % 1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform,
    % 6: Our FMS 2D 7: FMS hamming 8: FMS none
    % 9: Feature0 10: Feature1  11: Feature2 12: Feature3 13: Feature4 14: Feature5
    % 15: gmmRegistrationD2D 16: gmmRegistrationP2D 
    
    % 1 whichMethod 2: voxelSize 3: initialGuess 4: errorInDistance 5: errorInRotation 6: calculationTime 7: overlap
    
    
    % method_Of_interest = 6
    % voxelSize = 256;
    % initualGuess = 0;
    
    
%     journalOrdner = "/home/tim-external/dataFolder/journalPaperDatasets/";
    journalOrdner = "/home/tim-external/dataFolder/journalPaperDatasets/newDatasetsCreation/";

    % ordner = "highNoiseBigMotionKeller";
    % ordner = "highNoiseBigMotionValentin";
    % ordner = "noNoiseSmallMotionValentin";
    % ordner = "noNoiseSmallMotionKeller";
    % ordner = "onlyRotationNoNoiseValentin";
    % ordner = "onlyRotationNoNoiseKeller";

    for i = 1:numberOfCombinedDatasets
        if i == 1
            resultsExperiment = readmatrix(journalOrdner+ordner+"/"+"results.csv");

        else
            resultsExperiment = [resultsExperiment; readmatrix(journalOrdner+ordner+"/"+"results"+i+".csv")];

        end

    end

    









    
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
    validPercentage= size(resultListOfInterestWithoutNANs,1)/size(resultListOfInterest,1);
    display("percent of results not NAN: " + validPercentage)
    meanL2Error = mean(resultListOfInterestWithoutNANs(:,4));
    stdL2Error = std(resultListOfInterestWithoutNANs(:,4));
    display("mean: "+ meanL2Error)
    display("std: " + stdL2Error)
    
    meanAngleError = mean(resultListOfInterestWithoutNANs(:,5));
    stdAngleError = std(resultListOfInterestWithoutNANs(:,5));
    display("mean: "+ meanAngleError)
    display("std: " + stdAngleError)
    
    
    
    y1 = resultListOfInterestWithoutNANs(:,4);
    y2 = resultListOfInterestWithoutNANs(:,5);
    x1 = [zeros(size(resultListOfInterestWithoutNANs(:,7))),resultListOfInterestWithoutNANs(:,7)];
    x2 = [zeros(size(resultListOfInterestWithoutNANs(:,7))),initialGuessList(:,4)];
    x3 = [zeros(size(resultListOfInterestWithoutNANs(:,7))),initialGuessList(:,5)];
    b1 = x1\y1
    b2 = x2\y1
    b3 = x3\y2
    
    % b3 = resultListOfInterestWithoutNANs(:,7)\y
    % b4 = initialGuessList(:,4)\y
    % regression overlap
    figure(1)
    clf
    hold on
    if(usePoints)


       plot(resultListOfInterestWithoutNANs(:,7),resultListOfInterestWithoutNANs(:,4),'.')
       plot([0,max(resultListOfInterestWithoutNANs(:,7))],[b1(1),b1(1)+max(resultListOfInterestWithoutNANs(:,7))*b1(2)])
       grid on
       box on
    else

    
        values = hist3([resultListOfInterestWithoutNANs(:,7),resultListOfInterestWithoutNANs(:,4)],[sizeOfGrid sizeOfGrid]);
        imagesc(values.')
        colorbar
    
        if(size(resultListOfInterestWithoutNANs,1)>0)
            set(gca, 'XTick', [0:0.2:1]*sizeOfGrid, 'XTickLabel', [0:0.2:1]*round(max(resultListOfInterestWithoutNANs(:,7)),2)) % 10 ticks 
            set(gca, 'YTick', [0:0.2:1]*sizeOfGrid, 'YTickLabel', [0:0.2:1]*round(max(resultListOfInterestWithoutNANs(:,4)),2)) % 20 ticks
        end
    
        axis equal
        axis xy    
        set(gca,'ColorScale','log')
    end

    ylim([0 l2NormMax])

%  
%  
    
    pbaspect([2 1 1])

    if plotNumbers
        title(string(methodOfInterestToString(method_Of_interest))+ " L2 overlap regression: "+b1(2))
    else
        title(string(methodOfInterestToString(method_Of_interest))+ " L2 overlap regression")
    end
    
    nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/l2RegressionOverlap' +string(voxelSize) + string(ordner) + string(initualGuess)+ string(method_Of_interest);
    saveas(gcf,nameOfPdfFile, 'pdf' )
    addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf &',true);

    % system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');
    

%     figure(88)






    
    % set(gca, 'YScale', 'log')
    % regression initial guess
    figure(2)
    clf
    hold on 

    if(usePoints)

        plot(initialGuessList(:,4),resultListOfInterestWithoutNANs(:,4),'.')
        plot([0,max(initialGuessList(:,4))],[b2(1),b2(1)+max(initialGuessList(:,4))*b2(2)])
        grid on
        box on
    else


        values = hist3([initialGuessList(:,4),resultListOfInterestWithoutNANs(:,4)],[sizeOfGrid sizeOfGrid]);
        imagesc(values.')
        colorbar
    
        if(size(resultListOfInterestWithoutNANs,1)>0)
            set(gca, 'XTick', [0:0.2:1]*sizeOfGrid, 'XTickLabel', [0:0.2:1]*round(max(initialGuessList(:,4)),2)) % 10 ticks 
            set(gca, 'YTick', [0:0.2:1]*sizeOfGrid, 'YTickLabel', [0:0.2:1]*round(max(resultListOfInterestWithoutNANs(:,4)),2)) % 20 ticks
        end
    
        axis equal
        axis xy
    
        set(gca,'ColorScale','log')

    end

    ylim([0 l2NormMax])

   
    pbaspect([2 1 1])

    if plotNumbers
        title(string(methodOfInterestToString(method_Of_interest))+ " L2 initial guess regression: "+b2(2))
    else
        title(string(methodOfInterestToString(method_Of_interest))+ " L2 initial guess regression")
    end
    % set(gca, 'YScale', 'log')
    nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/l2RegressionInitialGuess' +string(voxelSize) + string(ordner) + string(initualGuess)+ string(method_Of_interest);
    saveas(gcf,nameOfPdfFile, 'pdf' )
    addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf &',true);

    % system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');
    
    
     if(isnan(meanL2Error))
        meanL2Error = -1;
        stdL2Error = -1;
        meanAngleError = -1;
        stdAngleError = -1;

     end


    figure(3)

    if plotNumbers
        label1 = char(append('L2 error ',string(meanL2Error) , '+-' , string(stdL2Error)));
        label2 = char(append('angleError ',string(meanAngleError) ,'+-', string(stdAngleError)));
    else
        label1 = char(append('L2 error '));
        label2 = char(append('angleError '));
    end

    % label1 = char(append('L2 error ',string(meanL2Error) , '+-' , string(stdL2Error)));
    % label2 = char(append('angleError ',string(meanAngleError) ,'+-', string(stdAngleError)));
    label3 = {label1 , label2};


    boxplot([resultListOfInterestWithoutNANs(:,4),resultListOfInterestWithoutNANs(:,5)],'Labels',label3)


    pbaspect([2 1 1])

    if plotNumbers
        title(string(methodOfInterestToString(method_Of_interest))+ " valid: " + string(validPercentage))
    else
        title(string(methodOfInterestToString(method_Of_interest)))
    end

    % title(string(methodOfInterestToString(method_Of_interest))+ " valid: " + string(validPercentage))
    set(gca, 'YScale', 'log')
    nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/boxplot' +string(voxelSize) + string(ordner) + string(initualGuess)+ string(method_Of_interest);
    saveas(gcf,nameOfPdfFile, 'pdf' )
    addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf &',true);

    % system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');
    
    figure(4)
    clf
    
    hold on 

    if(usePoints)

        plot(initialGuessList(:,5),resultListOfInterestWithoutNANs(:,5),'.')
        plot([0,max(initialGuessList(:,5))],[b3(1),b3(1)+max(initialGuessList(:,5))*b3(2)])
        grid on
        box on
    else
    
        values = hist3([initialGuessList(:,5),resultListOfInterestWithoutNANs(:,5)],[sizeOfGrid sizeOfGrid]);
        imagesc(values.')
        colorbar
    
        if(size(resultListOfInterestWithoutNANs,1)>0)
            set(gca, 'XTick', [0:0.2:1]*sizeOfGrid, 'XTickLabel', [0:0.2:1]*round(max(initialGuessList(:,5)),2)) % 10 ticks 
            set(gca, 'YTick', [0:0.2:1]*sizeOfGrid, 'YTickLabel', [0:0.2:1]*round(max(resultListOfInterestWithoutNANs(:,5)),2)) % 20 ticks
        end
    
        axis equal
        axis xy
    
        set(gca,'ColorScale','log')

    end


    ylim([0 ymax])



    title("angle initial guess regression steigung: "+b3(2))
    if plotNumbers
        title(string(methodOfInterestToString(method_Of_interest))+ " angle initial guess regression: " + b3(2))
    else
        title(string(methodOfInterestToString(method_Of_interest))+ " rotation error")
    end
    
    
    pbaspect([2 1 1])
    nameOfPdfFile = '/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdfResults/regressionAngle' +string(voxelSize) + string(ordner) + string(initualGuess)+ string(method_Of_interest);
    saveas(gcf,nameOfPdfFile, 'pdf' )
    addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf &',true);
    % system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');



end

