clc
%clear

%THIS FILE WORKS ONLY WITH PLOT ALL SOLUTIONS
ordnerSave = "/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdffigures/";
nameOfBatchFile = "batchfile.sh";


resampledDataForSphere1 = readmatrix("csvFiles/resampledVoxel1.csv");

% resultSize = size(resampledDataForSphere1,1);
sizeOfSpheres = sqrt(size(resampledDataForSphere1,1));

resampledDataForSphereResult1 =zeros(sizeOfSpheres,sizeOfSpheres);

for j = 1:sizeOfSpheres
    for i =1:sizeOfSpheres
            resampledDataForSphereResult1(j,i) = resampledDataForSphere1((i-1)*sizeOfSpheres+j);
    end
end

% currently not that interesting
if 1
    f = figure(4);
    clf
    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
    axis equal
    f.Position = [100 100 600 570];

else
    f = figure(4);
    clf

    imagesc((resampledDataForSphereResult1));
    axis image    
    f.Position = [100 100 600 570];
end



nameOfMap = "enhancedWrappedSphere";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);

% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);



%%


f = figure(6);
clf
correlationOfAngles = readmatrix("csvFiles/resultingCorrelation1D.csv");
xForPlot = 0:size(correlationOfAngles,1)-1;
xForPlot = xForPlot/size(correlationOfAngles,1)*360;
plot(xForPlot,correlationOfAngles)

%     axis image    
% f.Position = [100 100 600 250];
f.Position = [100 100 400 400];
grid on

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

ylabel("Correlation Value", 'Interpreter', 'latex')
xlabel("angle in degree", 'Interpreter', 'latex')




nameOfMap = "1DCorrelation";
nameOfFile = ordnerSave+nameOfMap;


saveas(gcf,nameOfFile , 'pdf');
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
%%
% resultSize = sizeOfSpheres;

f = figure(8);


clf
[Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
surf(Xplot,Yplot,(correlationMatrixShift2D),'edgecolor', 'none');
xlabel("x-axis", 'Interpreter', 'latex');
ylabel("y-axis", 'Interpreter', 'latex');

nameOfMap = "2DCorrelationWithCorrection";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');
saveas(gcf,nameOfFile , 'pdf');
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);


%%
dataInformation = readmatrix("csvFiles/dataForReadIn.csv");

numberOfSolutionsOverall = 0;
for i = 1:dataInformation(1)
    numberOfSolutionsOverall=numberOfSolutionsOverall+dataInformation(i+1);
end
cellSize = dataInformation(dataInformation(1)+2);

f = figure(8);
clf
[Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
surf(Xplot,Yplot,(correlationMatrixShift2D),'edgecolor', 'none');
xlabel("x-axis", 'Interpreter', 'latex');
ylabel("y-axis", 'Interpreter', 'latex');

hold on 
for currentTranslationNumber = 1:dataInformation(0+2)
    potentialTransformationReadIn = readmatrix("csvFiles/potentialTransformation"+(currentTranslationNumber-1)+".csv");
    transformationTMPSI = potentialTransformationReadIn(1:4,1:4);
    potentialTransformationReadIn(1,4) = potentialTransformationReadIn(1,4)/cellSize;
    potentialTransformationReadIn(2,4) = potentialTransformationReadIn(2,4)/cellSize;
    
    
    xOfInterest = -potentialTransformationReadIn(1,4)+ceil(resultSize/2);
    yOfInterest = -potentialTransformationReadIn(2,4)+ceil(resultSize/2);
    
    zForPlot=correlationMatrixShift2D(round(yOfInterest),round(xOfInterest));
    plot3(xOfInterest,yOfInterest,zForPlot,'r+')
end

nameOfMap = "2DCorrelationPeakDetectionFew";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);
% 
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
%%


figure(11)
clf

imagesc((magnitude1))
axis image

nameOfMap = "magnitude1";
nameOfFile = ordnerSave+nameOfMap;


set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);



figure(22)
clf

imagesc((magnitude2))
axis image

nameOfMap = "magnitude2";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);



figure(1)
clf

title('Voxel: '+string(1))

imagesc((voxelData1))
axis image

nameOfMap = "voxelData1";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);

% figure 2
figure(2)
clf

imagesc((voxelData2))
axis image


nameOfMap = "voxelData2";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);





figure(14)

sphere(1000);
ch = get(gca,'children');
set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
axis equal

nameOfMap = "2DSphere1";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);


figure(15)

sphere(1000);
ch = get(gca,'children');
set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult2,'edgecolor','none');
axis equal

nameOfMap = "2DSphere2";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);


figure(16)
imagesc((resampledDataForSphereResult1));
axis image

nameOfMap = "2DUnwrappedSphere1";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);


figure(17)
imagesc((resampledDataForSphereResult2));
axis image


nameOfMap = "2DUnwrappedSphere2";
nameOfFile = ordnerSave+nameOfMap;

set(groot,'defaultAxesTickLabelInterpreter','latex'); 
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);
addCommandToBatchfile(nameOfBatchFile,'pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);
