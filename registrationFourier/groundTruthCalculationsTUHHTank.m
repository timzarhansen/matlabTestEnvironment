clc
clear
% nameOfMethod = "_classical_slam_";
% nameOfMethod = "_dynamic_slam_4_0_";
% nameOfMethod = "_dead_reckoning_";
% nameOfMethod = "_dead_reckoning_wsm_";
% nameOfMethod = "_dynamic_slam_1_0_";

% nameOfDataset = "_circle";
% nameOfDataset = "_s_curve";
% nameOfDataset = "_squares";
% nameOfDataset = "_TEST1";
% nameOfDataset = "_valentinKeller";
% nameOfDataset = "_simulation";
nameOfMethodList = ["_classical_slam_"];
% nameOfMethodList = ["_dynamic_slam_4_0_","_dead_reckoning_","_classical_slam_"];
% nameOfMethodList = ["oceanLabExp1"];


% nameOfDatasetList = ["_circle","_s_curve","_squares"];
% nameOfDatasetList = ["_simulation","_circle","_s_curve","_valentinKeller","_squares"];
% nameOfDatasetList = ["_valentinKeller","_valentinOben"];
nameOfDatasetList = ["_simulation"];
% nameOfDatasetList = [""];
for nameOfMethod = nameOfMethodList
    for nameOfDataset = nameOfDatasetList
%%
clearvars -except nameOfMethod nameOfDataset nameOfMethodList nameOfDatasetList





inputPosition = readmatrix("csvFiles/IROSResults/positionEstimationOverTime" + nameOfDataset+nameOfMethod +".csv");
inputPosition = inputPosition(1:end-4*70,1:4);
inputGroundTruth = readmatrix("csvFiles/IROSResults/groundTruthOverTime" + nameOfDataset+nameOfMethod +".csv");
inputGroundTruth = inputGroundTruth(1:end-4*70,1:4);

how_many_skip = 800;
inputPosition = inputPosition(4*how_many_skip+1:end,:);
inputGroundTruth = inputGroundTruth(4*how_many_skip+1:end,: );



firstMatrixGT = inputGroundTruth(1:4,1:4);
firstMatrixSLAM = inputPosition(1:4,1:4);


numberOfVertices = size(inputGroundTruth,1)/4
for i = 1:numberOfVertices

    gtMatrix(i,:,:) = firstMatrixGT\inputGroundTruth((i-1)*4+1:(i-1)*4+4,1:4);
    slamMatrix(i,:,:) = firstMatrixSLAM\inputPosition((i-1)*4+1:(i-1)*4+4,1:4);
    if nameOfDataset == "_circle" ||nameOfDataset == "_s_curve" || nameOfDataset == "_squares"
        gtMatrix(i,1,4) = -gtMatrix(i,1,4); %% needed at TUHH tank error
    end


    xPosSlam(i) = slamMatrix(i,1,4);
    yPosSlam(i) = slamMatrix(i,2,4);
    angleSLAM(i) = atan2(slamMatrix(i,2,1),slamMatrix(i,1,1));


    xPosGT(i) = gtMatrix(i,1,4);
    yPosGT(i) = gtMatrix(i,2,4);  
    angleGT(i) = atan2(gtMatrix(i,2,1),gtMatrix(i,1,1));
    % display("next")   
end



% differenceX = (xPosSlam-xPosGT);
% differenceY = (yPosSlam-yPosGT);
% mean(differenceX)
% mean(differenceY)
% mean(vecnorm( [differenceX;differenceY])')

%%

%calcError(eye(4),gtMatrix,slamMatrix)

optimizeParameters = [0,0,0];
Z = fminsearch(@(v) calcError(v(1),v(2),v(3),gtMatrix,slamMatrix),optimizeParameters);
%Z = [0 0 0];


% Z = [0.3278   -0.5767   -0.0371];
%%
calcError(Z(1),Z(2),Z(3),gtMatrix,slamMatrix)
calcError(0,0,Z(3),gtMatrix,slamMatrix)
%%
matrixforRelativeError = eye(4);
matrixforRelativeError(1,4) = Z(1);
matrixforRelativeError(2,4) = Z(2);
matrixforRelativeError(1,1) = cos(Z(3));
matrixforRelativeError(2,2) = cos(Z(3));
matrixforRelativeError(1,2) = -sin(Z(3));
matrixforRelativeError(2,1) = sin(Z(3));

for i = 1:numberOfVertices

    %gtMatrix(i,:,:) = inv(firstMatrixGT)*inputGroundTruth((i-1)*4+1:(i-1)*4+4,1:4);
    %slamMatrix(i,:,:) = inputPosition((i-1)*4+1:(i-1)*4+4,1:4);
    tmpMatrix = matrixforRelativeError*squeeze(gtMatrix(i,:,:));
    xPosSlam(i) = slamMatrix(i,1,4);
    yPosSlam(i) = slamMatrix(i,2,4);

    xPosGT(i) = tmpMatrix(1,4);%+0.3348
    yPosGT(i) = tmpMatrix(2,4);%+0.1960
    
    % display("next")
end
%% calculate the relative angle error
k = 1;
windowSize = 400;
for i = windowSize/2:numberOfVertices-windowSize/2
    correctioni = windowSize/2-1;
    xPosSLAM1 = slamMatrix(i-correctioni,1,4);
    yPosSLAM1 = slamMatrix(i-correctioni,2,4);
    xPosSLAM2 = slamMatrix(i+correctioni,1,4);
    yPosSLAM2 = slamMatrix(i+correctioni,2,4);

    tmpMatrix1 = matrixforRelativeError*squeeze(gtMatrix(i-correctioni,:,:));
    tmpMatrix2 = matrixforRelativeError*squeeze(gtMatrix(i+correctioni,:,:));

    xPosGT1 = tmpMatrix1(1,4);
    yPosGT1 = tmpMatrix1(2,4);
    xPosGT2 = tmpMatrix2(1,4);
    yPosGT2 = tmpMatrix2(2,4);


    errorAngle1 = atan2(yPosSLAM1-yPosSLAM2,xPosSLAM1-xPosSLAM2);
    errorAngle2 = atan2(yPosGT1-yPosGT2,xPosGT1-xPosGT2);
    relativeAngleError(k) = angleDiff(errorAngle1,errorAngle2 );
    k =  k+1;
end

%% calculate the absolute angle error
matrixforAbsoluteError = eye(4);
matrixforAbsoluteError(1,4) = 0;
matrixforAbsoluteError(2,4) = 0;
matrixforAbsoluteError(1,1) = cos(Z(3));
matrixforAbsoluteError(2,2) = cos(Z(3));
matrixforAbsoluteError(1,2) = -sin(Z(3));
matrixforAbsoluteError(2,1) = sin(Z(3));
k = 1;
for i = windowSize/2:numberOfVertices-windowSize/2
    correctioni = windowSize/2-1;
    xPosSLAM1 = slamMatrix(i-correctioni,1,4);
    yPosSLAM1 = slamMatrix(i-correctioni,2,4);
    xPosSLAM2 = slamMatrix(i+correctioni,1,4);
    yPosSLAM2 = slamMatrix(i+correctioni,2,4);

    tmpMatrix1 = matrixforAbsoluteError*squeeze(gtMatrix(i-correctioni,:,:));
    tmpMatrix2 = matrixforAbsoluteError*squeeze(gtMatrix(i+correctioni,:,:));

    xPosGT1 = tmpMatrix1(1,4);
    yPosGT1 = tmpMatrix1(2,4);
    xPosGT2 = tmpMatrix2(1,4);
    yPosGT2 = tmpMatrix2(2,4);


    errorAngle1 = atan2(yPosSLAM1-yPosSLAM2,xPosSLAM1-xPosSLAM2);
    errorAngle2 = atan2(yPosGT1-yPosGT2,xPosGT1-xPosGT2);
    absoluteAngleError(k) = angleDiff(errorAngle1,errorAngle2 );
    k =  k+1;
end

%%

figure(3)
clf
magnitudeMapInput = readmatrix("csvFiles/IROSResults/currentMap" + nameOfDataset + nameOfMethod +".csv");
N=sqrt(size(magnitudeMapInput,1));
for j =1:N
    for i =1:N
        magnitudeMap(i,j) = magnitudeMapInput((i-1)*N+j);
    end
end
rotationAngleImage = 0;
rotationGTSystem = 0;



sizePixel = 10/512;
posXZero = 95;
posYZero = 85;
start_number = 100;
end_number = 440;
imageShiftX = 0;
imageShiftY = 0;

if nameOfDataset == "_valentinKeller"
    sizePixel = 45/512;
end

if nameOfDataset == "_simulation"
    sizePixel = 80/512;
    posXZero = 190;
    posYZero = 398;
    start_number = 1;
    end_number = 420;
    rotationAngleImage = 3;

    if nameOfMethod == "_dynamic_slam_4_0_"
        imageShiftX = 30;
        imageShiftY = 3;
    end

        if nameOfMethod == "_dead_reckoning_"
        posYZero = 415;
    end
end

if nameOfDataset == "_circle"
        imageShiftX = 30;
        imageShiftY = 3;
        start_number = 140;
        end_number = 430;
        posXZero = 75;
        posYZero = 58;
        
        if nameOfMethod == "_dead_reckoning_"
            imageShiftX = 20;
            imageShiftY = 35;
            posXZero = 50;
            posYZero = 58;
        end


        
        if nameOfMethod == "_classical_slam_"
            posXZero = 70;
            posYZero = 36;
        end

        if nameOfMethod == "_dynamic_slam_4_0_"
            posXZero = 72;
            posYZero = 37;
        end


end

if nameOfDataset == "_s_curve"


        imageShiftX = 30;
        imageShiftY = -45;
        start_number = 90;
        end_number = 420;
        posXZero = 88;
        posYZero = 60;



        if nameOfMethod == "_dead_reckoning_"
            rotationGTSystem = 1.12;
        end


end

if nameOfDataset == "_squares"

    start_number = 110;
    end_number = 440;
    imageShiftX = 20;
    imageShiftY = 20;

        if nameOfMethod == "_dynamic_slam_4_0_"
            posXZero = 100;
            posYZero = 98;
        end

        if nameOfMethod == "_dead_reckoning_"
            rotationGTSystem = 0.775;
        end
end

testImage = magnitudeMap;
xPosGT = firstMatrixGT(1,4);
yPosGT = firstMatrixGT(2,4);

xDesMidPoint = 1.8;
yDesMidPoint = 1.8;
diffX = xPosGT-xDesMidPoint;
diffY = yPosGT-yDesMidPoint;
diffX=diffX/sizePixel;
diffY=diffY/sizePixel;

% testImage = flipdim(rot90(rot90(testImage)),2);
testImage = imrotate(testImage,rotationAngleImage);
testImage = circshift(testImage,+round(diffX)+imageShiftX,2);
testImage = circshift(testImage,+round(diffY)+imageShiftY,1);
testImage = testImage(start_number:end_number,start_number*1.1:end_number*0.9);





%calculation of path


    quaterDegreeMatrix = eye(4);
    quaterDegreeMatrix(1:3,1:3) = rotationMatrix(0,0,rotationGTSystem)*rotationMatrix(0,pi,0)*rotationMatrix(0,0,pi/2);

for i = 1:numberOfVertices
    tmpMatrixGT = quaterDegreeMatrix*matrixforRelativeError*squeeze(gtMatrix(i,:,:));
    tmpMatrixSLAM =  quaterDegreeMatrix*squeeze(slamMatrix(i,:,:));

    pictureGTPlotX(i) = tmpMatrixGT(1,4);%+0.3348
    pictureGTPlotY(i) = tmpMatrixGT(2,4);%+0.1960
    pictureSLAMPlotX(i) = tmpMatrixSLAM(1,4);
    pictureSLAMPlotY(i) = tmpMatrixSLAM(2,4);

    pictureSLAMPlotX(i) = pictureSLAMPlotX(i) /sizePixel + posXZero+xPosGT/sizePixel;
    pictureSLAMPlotY(i) = pictureSLAMPlotY(i) /sizePixel + posYZero+yPosGT/sizePixel;

    pictureGTPlotX(i) = pictureGTPlotX(i)/sizePixel+posXZero+xPosGT/sizePixel;
    pictureGTPlotY(i) = pictureGTPlotY(i)/sizePixel+posYZero+yPosGT/sizePixel;

    % display("next")
end



removeLastRows = 300;
pictureSLAMPlotX = pictureSLAMPlotX(1:end-removeLastRows);
pictureSLAMPlotY = pictureSLAMPlotY(1:end-removeLastRows);

pictureGTPlotX = pictureGTPlotX(1:end-removeLastRows);
pictureGTPlotY = pictureGTPlotY(1:end-removeLastRows);






clf 
hold on




imagesc(testImage)
plot(pictureGTPlotX,pictureGTPlotY,'color','r')
plot(pictureSLAMPlotX,pictureSLAMPlotY,'color','g')
axis equal
axis image



xlim([1 size(testImage,1)])
ylim([1 size(testImage,2)])


if nameOfDataset == "_circle"
    xlim([1 size(testImage,2)])
    ylim([1 size(testImage,1)])
end
if nameOfDataset == "_squares"
    xlim([1 size(testImage,2)])
    ylim([1 size(testImage,1)])
end

if nameOfDataset == "_s_curve"
    xlim([1 size(testImage,2)])
    ylim([1 size(testImage,1)])
end
if nameOfDataset == "_simulation"
    xlim([1 size(testImage,2)])
    ylim([1 size(testImage,1)])
end



if nameOfDataset == "_simulation"
    ax = gca;
    tmp = ax.XTick
    xticklabels({tmp/5 })
    tmp = ax.YTick
    yticklabels({tmp/5 })
else
    ax = gca;
    tmp = ax.XTick
    xticklabels({tmp/50})
    tmp = ax.YTick
    yticklabels({tmp/50})
end






ylabel("m", 'Interpreter', 'latex')
xlabel("m", 'Interpreter', 'latex')
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
% set(gca,'YTickLabel',[]);
% set(gca,'XTickLabel',[]);
%pbaspect([1 1 1])
nameOfPdfFile = '/home/ws/matlab/registrationFourier/csvFiles/IROSResults/images/' + nameOfDataset + nameOfMethod;
saveas(gcf,nameOfPdfFile, 'pdf' )

% system ('gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/ebook -dNOPAUSE -dQUIET -dBATCH -sOutputFile=' +nameOfPdfFile +'.pdf '+ nameOfPdfFile +'.pdf');


system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');





%%


% figure(2)
% clf
% hold on
% plot(xPosSlam)
% 
% plot(xPosGT)
% 
% figure(1)
% clf
% hold on
% 
% plot(yPosGT,xPosGT)
% plot(yPosSlam,xPosSlam)
% axis equal
%%
fid = fopen('errorValues.txt', 'a+');
% fprintf(fid, nameOfDataset + nameOfMethod+'\n');


fprintf(fid,nameOfDataset + nameOfMethod+'\n')
fprintf(fid,"Absolute mean angle error: "+ string(mean(abs(absoluteAngleError)))+'\n')
fprintf(fid,"Absolute std angle error: "+ string(std(abs(absoluteAngleError)))+'\n')
fprintf(fid,"Absolute mean l2 error: "+ string(calcError(0,0,Z(3),gtMatrix,slamMatrix))+'\n')
fprintf(fid,"Absolute std l2 error: "+ string(calcStd(0,0,Z(3),gtMatrix,slamMatrix))+'\n')
fprintf(fid,"Relative mean angle error: "+ string(mean(abs(relativeAngleError)))+'\n')
fprintf(fid,"Relative std angle error: "+ string(std(abs(relativeAngleError)))+'\n')
fprintf(fid,"Relative mean l2 error: "+ string(calcError(Z(1),Z(2),Z(3),gtMatrix,slamMatrix))+'\n')
fprintf(fid,"Relative std l2 error: "+ string(calcStd(Z(1),Z(2),Z(3),gtMatrix,slamMatrix))+'\n')
fprintf(fid,"Angle mean error: "+ string(mean(abs(angleSLAM)))+'\n')
fprintf(fid,"Angle std error: "+ string(std(abs(angleSLAM)))+'\n')
fclose(fid);

    end
end

%%

function normResult = calcError(x,y,alpha,gtMatrix,slamMatrix)
    matrixToBeOptimized = eye(4);
    matrixToBeOptimized(1,4) = x;
    matrixToBeOptimized(2,4) = y;
    matrixToBeOptimized(1,1) = cos(alpha);
    matrixToBeOptimized(2,2) = cos(alpha);
    matrixToBeOptimized(1,2) = -sin(alpha);
    matrixToBeOptimized(2,1) = sin(alpha);
    currentSize = size(gtMatrix,1);
    normTMP = zeros(currentSize,1);
    for i = 1:currentSize
         tmpMatrix1 = matrixToBeOptimized*squeeze(gtMatrix(i,:,:));
         tmpMatrix2 = squeeze(slamMatrix(i,:,:));
         normTMP(i,1) = norm([tmpMatrix1(1,4)-tmpMatrix2(1,4);tmpMatrix1(2,4)-tmpMatrix2(2,4)]);
    end
    normResult = mean(abs(normTMP));

end

function normResult = calcStd(x,y,alpha,gtMatrix,slamMatrix)
    matrixToBeOptimized = eye(4);
    matrixToBeOptimized(1,4) = x;
    matrixToBeOptimized(2,4) = y;
    matrixToBeOptimized(1,1) = cos(alpha);
    matrixToBeOptimized(2,2) = cos(alpha);
    matrixToBeOptimized(1,2) = -sin(alpha);
    matrixToBeOptimized(2,1) = sin(alpha);
    currentSize = size(gtMatrix,1);
    normTMP = zeros(currentSize,1);
    for i = 1:currentSize
         tmpMatrix1 = matrixToBeOptimized*squeeze(gtMatrix(i,:,:));
         tmpMatrix2 = squeeze(slamMatrix(i,:,:));
         normTMP(i,1) = norm([tmpMatrix1(1,4)-tmpMatrix2(1,4);tmpMatrix1(2,4)-tmpMatrix2(2,4)]);
    end
    normResult = std(abs(normTMP));

end
% old     0.0368 4.0 + s courve good 











