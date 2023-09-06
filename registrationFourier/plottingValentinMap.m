clc
clear
figure(3)
clf

nameOfDataset = "_valentinKeller_dynamic_slam_4_0_";

inputPosition = readmatrix("csvFiles/IROSResults/positionEstimationOverTime" + nameOfDataset +".csv");
inputPosition = inputPosition(1:end-4*70,1:4);

how_many_skip = 800;
inputPosition = inputPosition(4*how_many_skip+1:end,:);


firstMatrixSLAM = inputPosition(1:4,1:4);

numberOfVertices = size(inputPosition,1)/4
for i = 1:numberOfVertices

    slamMatrix(i,:,:) = firstMatrixSLAM\inputPosition((i-1)*4+1:(i-1)*4+4,1:4);

    xPosSlam(i) = slamMatrix(i,1,4);
    yPosSlam(i) = slamMatrix(i,2,4);
    angleSLAM(i) = atan2(slamMatrix(i,2,1),slamMatrix(i,1,1));

end





magnitudeMapInput = readmatrix("csvFiles/IROSResults/currentMap" + nameOfDataset +".csv");
N=sqrt(size(magnitudeMapInput,1));
for j =1:N
    for i =1:N
        magnitudeMap(i,j) = magnitudeMapInput((i-1)*N+j);
    end
end

rotationAngleImage = 10;
rotationpath = -0.5;



sizePixel = 45/512;
posXZero = 255;
posYZero = 265;
start_number = 1;
end_number = 512;
imageShiftX = -60;
imageShiftY = -35;

testImage = magnitudeMap;


xDesMidPoint = 0.0;
yDesMidPoint = 0.0;
diffX = 0-xDesMidPoint;
diffY = 0-yDesMidPoint;
diffX=diffX/sizePixel;
diffY=diffY/sizePixel;

% testImage = flipdim(rot90(rot90(testImage)),2);
testImage = imrotate(testImage,rotationAngleImage);
testImage = circshift(testImage,+round(diffX)+imageShiftX,2);
testImage = circshift(testImage,+round(diffY)+imageShiftY,1);
testImage = testImage(start_number:end_number,start_number*1.1:end_number*0.9);





%calculation of path


    quaterDegreeMatrix = eye(4);
    quaterDegreeMatrix(1:3,1:3) = rotationMatrix(0,0,rotationpath)*rotationMatrix(0,pi,0)*rotationMatrix(0,0,pi/2);

for i = 1:numberOfVertices
    tmpMatrixSLAM =  quaterDegreeMatrix*squeeze(slamMatrix(i,:,:));

    pictureSLAMPlotX(i) = tmpMatrixSLAM(1,4);
    pictureSLAMPlotY(i) = tmpMatrixSLAM(2,4);

    pictureSLAMPlotX(i) = pictureSLAMPlotX(i) /sizePixel + posXZero;
    pictureSLAMPlotY(i) = pictureSLAMPlotY(i) /sizePixel + posYZero;

    % display("next")
end



removeLastRows = 300;
pictureSLAMPlotX = pictureSLAMPlotX(1:end-removeLastRows);
pictureSLAMPlotY = pictureSLAMPlotY(1:end-removeLastRows);






clf 
hold on




imagesc(testImage)
plot(pictureSLAMPlotX,pictureSLAMPlotY,'color','g')
axis equal
axis image



xlim([1 size(testImage,2)])
ylim([1 size(testImage,1)])



ax = gca;
tmp = ax.XTick
% set(gca,'XTickLabel',round(tmp*sizePixel))
xticklabels(round(tmp*sizePixel))
% ax.YAxis.TickLabelFormat = '%.1f';
tmp = ax.YTick

yticklabels(round(tmp*sizePixel ))





ylabel("m", 'Interpreter', 'latex')
xlabel("m", 'Interpreter', 'latex')
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
% set(gca,'YTickLabel',[]);
% set(gca,'XTickLabel',[]);
%pbaspect([1 1 1])
nameOfPdfFile = '/home/tim-external/Documents/matlabTestEnvironment/registrationFourier/csvFiles/IROSResults/images/' + nameOfDataset;
saveas(gcf,nameOfPdfFile, 'pdf' )

% system ('gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/ebook -dNOPAUSE -dQUIET -dBATCH -sOutputFile=' +nameOfPdfFile +'.pdf '+ nameOfPdfFile +'.pdf');


system('pdfcrop ' + nameOfPdfFile + '.pdf '+nameOfPdfFile+ '.pdf');