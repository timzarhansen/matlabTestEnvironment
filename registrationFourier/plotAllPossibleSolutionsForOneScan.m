clc
clear
%interesting


%Difficult success number 3
%whichKeyframe = 11
%nameOfFolder = '/home/tim-external/dataFolder/gazeboCorrectedUnevenAnglesPCLs_4_100/';


% classical success 2
whichKeyframe = 24;%5
nameOfFolder = '/home/tim-external/dataFolder/gazeboCorrectedUnevenAnglesPCLs_4_100/';

% difficult failure
%whichKeyframe =37;%5
%nameOfFolder = '/home/tim-external/dataFolder/gazeboCorrectedUnevenAnglesPCLs_4_100/';

%whichKeyframe = 1;%5
%nameOfFolder = '/home/tim-external/dataFolder/gazeboCorrectedEvenAnglesPCLs_2_75/';
%nameOfFolder = '/home/tim-external/dataFolder/gazeboCorrectedUnevenAnglesPCLs_4_100/';
%nameOfFolder = '/home/tim-external/dataFolder/newStPereDatasetCorrectionOnly/';
firstScan=['pclKeyFrame',num2str(whichKeyframe),'.pcd'];
secondScan =['pclKeyFrame',num2str(whichKeyframe+1),'.pcd'];

command =['rosrun underwaterslam registrationOfTwoPCLs ',nameOfFolder,firstScan,' ',nameOfFolder,secondScan];
%system(cmdStr);
system(command);

%%

set(groot,'defaultAxesTickLabelInterpreter','latex');  

magnitudeFFTW1 = readmatrix("csvFiles/magnitudeFFTW1.csv");
N=sqrt(size(magnitudeFFTW1,1));

phaseFFTW1 = readmatrix("csvFiles/phaseFFTW1.csv");
voxelDataUsed1 = readmatrix("csvFiles/voxelDataFFTW1.csv");

magnitudeFFTW2 = readmatrix("csvFiles/magnitudeFFTW2.csv");
phaseFFTW2 = readmatrix("csvFiles/phaseFFTW2.csv");
voxelDataUsed2 = readmatrix("csvFiles/voxelDataFFTW2.csv");
%fftwResult = complex(realFFTW,imaginaryFFTW);
%fftwResult = (reshape(fftwResult,N,N,N));
magnitude1 =zeros(N,N);
phase1 =zeros(N,N);
voxelData1 =zeros(N,N);
magnitude2 =zeros(N,N);
phase2 =zeros(N,N);
voxelData2 =zeros(N,N);
for j =1:N
    for k =1:N
        magnitude1(j,k) = magnitudeFFTW1(j*N-N+k);
        phase1(j,k) = phaseFFTW1(j*N-N+k);
        voxelData1(j,k) = voxelDataUsed1((j-1)*N+k);
        magnitude2(j,k) = magnitudeFFTW2(j*N-N+k);
        phase2(j,k) = phaseFFTW2(j*N-N+k);
        voxelData2(j,k) = voxelDataUsed2((j-1)*N+k);
    end
end


magnitude1=fftshift(magnitude1);
phase1=fftshift(phase1);
magnitude2=fftshift(magnitude2);
phase2=fftshift(phase2);

figure(1)
clf
imagesc(squeeze(magnitude1));

%title('Magnitude first PCL Voxel:', 'Interpreter', 'latex')
axis image
box on 
pbaspect([1 1 1])
saveas(gcf, 'outputPDFs/magnitudeScan1', 'pdf')
system('pdfcrop outputPDFs/magnitudeScan1.pdf outputPDFs/magnitudeScan1.pdf');


figure(2)
clf
imagesc((voxelData1))
axis image
%title('First PointCloud as Voxel:', 'Interpreter', 'latex')
% figure 2
pbaspect([1 1 1])
saveas(gcf, 'outputPDFs/voxelData1', 'pdf')
system('pdfcrop outputPDFs/voxelData1.pdf outputPDFs/voxelData1.pdf');

figure(3)

imagesc(squeeze(magnitude2));


%imshow(magnitude2(:,:,size(magnitude2,1)/2));
%title('Magnitude second PCL Voxel:', 'Interpreter', 'latex')
axis image
pbaspect([1 1 1])
saveas(gcf, 'outputPDFs/magnitudeScan2', 'pdf')
system('pdfcrop outputPDFs/magnitudeScan2.pdf outputPDFs/magnitudeScan2.pdf');

figure(4)
imagesc((voxelData2))
axis image
%title('Second PointCloud as Voxel:', 'Interpreter', 'latex')
pbaspect([1 1 1])
saveas(gcf, 'outputPDFs/voxelData2', 'pdf')
system('pdfcrop outputPDFs/voxelData2.pdf outputPDFs/voxelData2.pdf');
%%
resampledDataForSphere1 = readmatrix("csvFiles/resampledVoxel1.csv");
resampledDataForSphere2 = readmatrix("csvFiles/resampledVoxel2.csv");

resampledDataForSphereResult1 =zeros(N,N);
resampledDataForSphereResult2 =zeros(N,N);
for i = 1:N
    for j =1:N
            resampledDataForSphereResult1(i,j) = resampledDataForSphere1((i-1)*N+j);
            resampledDataForSphereResult2(i,j) = resampledDataForSphere2((i-1)*N+j);
    end
end

% currently not that interesting
if 1
    figure(5)
    clf

    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult2,'edgecolor','none');
    axis equal
    %title('Second Resampled:', 'Interpreter', 'latex')
    pbaspect([1 1 1])
    saveas(gcf, 'outputPDFs/resampledDataForSphereResult2', 'pdf')
    system('pdfcrop outputPDFs/resampledDataForSphereResult2.pdf outputPDFs/resampledDataForSphereResult2.pdf');
    figure(6)
    clf
    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
    axis equal
    %title('first Resampled:', 'Interpreter', 'latex')
    pbaspect([1 1 1])
    saveas(gcf, 'outputPDFs/resampledDataForSphereResult1', 'pdf')
    system('pdfcrop outputPDFs/resampledDataForSphereResult1.pdf outputPDFs/resampledDataForSphereResult1.pdf');
else
    figure(5)
    clf
    imagesc((resampledDataForSphereResult2));
    axis image
        %title('second Resampled:', 'Interpreter', 'latex')
    pbaspect([1 1 1])
    saveas(gcf, 'outputPDFs/resampledDataForSphereResult2', 'pdf')
    system('pdfcrop outputPDFs/resampledDataForSphereResult2.pdf outputPDFs/resampledDataForSphereResult2.pdf');
    figure(6)
    clf
    imagesc((resampledDataForSphereResult1));
    axis image
    pbaspect([1 1 1])
        %title('first Resampled:', 'Interpreter', 'latex')
    saveas(gcf, 'outputPDFs/resampledDataForSphereResult1', 'pdf')
    system('pdfcrop outputPDFs/resampledDataForSphereResult1.pdf outputPDFs/resampledDataForSphereResult1.pdf');
end

%%
figure(7)
correlationOfAngles = readmatrix("csvFiles/resultingCorrelation1D.csv");

anglesX = linspace(0,2*pi,size(correlationOfAngles,1));
[psor,lsor] = findpeaks(correlationOfAngles);
plot(anglesX,correlationOfAngles)
for i=1:size(psor,1)
    text(anglesX(lsor(i))+0.15,psor(i)+1,[num2str(round(anglesX(lsor(i)),2)) ], 'Interpreter', 'latex')
end
grid on
xlim([-0.2 2*pi+0.2])
ylim([min(correlationOfAngles)-10 max(correlationOfAngles)+10])
%title('Correlation of different Angles', 'Interpreter', 'latex')
box on 
xlabel("angle in rad", 'Interpreter', 'latex');
ylabel("Correlation Value", 'Interpreter', 'latex');
pbaspect([1 1 1])
saveas(gcf, 'outputPDFs/resultingCorrelation1DAngle', 'pdf')
system('pdfcrop outputPDFs/resultingCorrelation1DAngle.pdf outputPDFs/resultingCorrelation1DAngle.pdf');

%%




%%

dataInformation = readmatrix("csvFiles/dataForReadIn.csv");

numberOfSolutions = dataInformation(1);
bestSolution = dataInformation(2);





% [M,I] = max(correlationMatrixShift2D, [], "all", "linear");
% [dim1, dim2, dim3] = ind2sub(size(correlationMatrixShift2D),I)

%correlationMatrixShift2D  = squeeze(correlationMatrixShift3D(65,:,:));
figure(8)
clf

for i = 1:numberOfSolutions
        subplot(2,2,i)
    correlationMatrixShift1D = readmatrix(['csvFiles/resultingCorrelationShift' num2str(i-1) '.csv']);
    resultSize = nthroot(length(correlationMatrixShift1D),2);
    %A = reshape(results,resultSize,resultSize);
    correlationMatrixShift2D = reshape(correlationMatrixShift1D,resultSize,resultSize);
    imagesc(correlationMatrixShift2D);
    [Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
    %surf(Xplot,Yplot,(correlationMatrixShift2D),'edgecolor', 'none');
    %xlabel("x-axis", 'Interpreter', 'latex');
    %ylabel("y-axis", 'Interpreter', 'latex');
    
    %title('Correlation Surface of  Best Performing Match', 'Interpreter', 'latex')
    box on 
    title(num2str(i), 'Interpreter', 'latex')
    pbaspect([1 1 1])
end
%set(gcf,'units','points','position',[0,0,500,500])

saveas(gcf, 'outputPDFs/2dCorrelation', 'pdf')
system('pdfcrop outputPDFs/2dCorrelation.pdf outputPDFs/2dCorrelation.pdf');





figure(9)
clf


for i = 1:numberOfSolutions
    subplot(2,2,i)

    pointCloudResult1 = pcread(['csvFiles/resulting' num2str(i-1) 'PCL1.pcd']);
    pointCloudResult2 = pcread(['csvFiles/resulting' num2str(i-1) 'PCL2.pcd']);
    tform = eye(4);
    tform(1:3,1:3) = rotationMatrix(0,0,0);
    tform(1,4)=0;%x value
    tform(2,4)=0;%y value
    tform=affine3d(tform');
    pointCloudResult1 = pctransform(pointCloudResult1,tform);
    
    
    
    pcshowpair(pointCloudResult1, pointCloudResult2)
    
    
    view(90,-90)
    set(gca,'color','w', 'XColor','k', 'YColor','k');
    set(gcf,'color','w');
    
%     xlabel('X-Axis in m', 'Interpreter', 'latex')
%     ylabel('Y-Axis in m', 'Interpreter', 'latex')
    title(num2str(i), 'Interpreter', 'latex')

    box on


end
saveas(gcf, 'outputPDFs/matchedPointcloudsAll', 'pdf')
system('pdfcrop outputPDFs/matchedPointcloudsAll.pdf outputPDFs/matchedPointcloudsAll.pdf');
%%


transformation1 = readmatrix([nameOfFolder , 'position',num2str(whichKeyframe),'.csv']); 
transformation2 = readmatrix([nameOfFolder , 'position',num2str(whichKeyframe+1),'.csv']);
translation1=transformation1(1:3);
translation2=transformation2(1:3);
rotation1 = reshape(transformation1(4:end),[3,3]);
rotation2 = reshape(transformation2(4:end),[3,3]);
resultingTranslationDiff = -transformation1(1:3);
resultingRotationDiff = transformation2(4:6)-transformation1(4:6);

completeTransformation1 = eye(4);
completeTransformation2 = eye(4);

completeTransformation1(1:3,1:3)=rotation1;
completeTransformation2(1:3,1:3)=rotation2;
completeTransformation1(1,4)=transformation1(1);
completeTransformation1(2,4)=transformation1(2);
completeTransformation2(1,4)=transformation2(1);
completeTransformation2(2,4)=transformation2(2);

GTTransformationDiff = inv(completeTransformation1) *completeTransformation2;
rpyTMP = anglesR(GTTransformationDiff(1:3,1:3),'xyz')/180*pi;




testTransformation = eye(4);
testTransformation(1:3,1:3) = rotationMatrix(pi,0,0);
%testTransformation(1:3,1:3) = rotationMatrix(0,0,pi);%*rotationMatrix(pi,0,0);


for i = 1:numberOfSolutions
    resultingTransformationOfScan = readmatrix(['csvFiles/resultingTransformation' num2str(i-1) '.csv']);
    resultingGTTransformation = testTransformation*inv(GTTransformationDiff);
    i
    resultingYawDiff = atan2(sin(rpyTMP(3) - resultingTransformationOfScan(6)), cos(rpyTMP(3) - resultingTransformationOfScan(6)))
    %atan2(sin(rpyTMP(3) - resultingTransformationOfScan(6)), cos(rpyTMP(3) - resultingTransformationOfScan(6)))
    translationDiff = resultingTransformationOfScan(1:2)-resultingGTTransformation(1:2,4)
    %resultingTransformationOfScan(1:2)-resultingGTTransformation(1:2,4)




end
% translationDiffX = resultingTransformationOfScan(1)-resultingGTTransformation(2,4)
% translationDiffY = resultingTransformationOfScan(2)-resultingGTTransformation(1,4)



% resultingTranslationDiff
% %resultingRotationDiff
% 
% resultingTransformationOfScan = readmatrix("resultingTransformation.csv")
% 
% 
% display("difference Rotation:")
% 
% resultingRotationDiff(3)-resultingTransformationOfScan(6)
% 
% display("difference Translation:")
% 
% min(abs(resultingTranslationDiff(1)+resultingTransformationOfScan(2)),abs(resultingTranslationDiff(1)-resultingTransformationOfScan(2)))
% min(abs(resultingTranslationDiff(2)+resultingTransformationOfScan(1)),abs(resultingTranslationDiff(2)-resultingTransformationOfScan(1)))
% 
% 
%%


if 1

    for i = 1:numberOfSolutions
        resultingTransformationOfScan = readmatrix(['csvFiles/resultingTransformation' num2str(i-1) '.csv']);
        resultingYawDiff(i) = abs(atan2(sin(rpyTMP(3) - resultingTransformationOfScan(6)), cos(rpyTMP(3) - resultingTransformationOfScan(6))));
    end
    [minimumInitialGuess,indexInitialGuess] = min(resultingYawDiff);
    figure (10)
    clf

    pointCloudResult1 = pcread(['csvFiles/resulting' num2str(indexInitialGuess-1) 'PCL1.pcd']);
    pointCloudResult2 = pcread(['csvFiles/resulting' num2str(indexInitialGuess-1) 'PCL2.pcd']);
    tform = eye(4);
    tform(1:3,1:3) = rotationMatrix(0,0,pi/2);
    tform(1,4)=0;%x value
    tform(2,4)=0;%y value
    tform=affine3d(tform');
    pointCloudResult1 = pctransform(pointCloudResult1,tform);
    pointCloudResult2 = pctransform(pointCloudResult2,tform);
    tform = eye(4);
    tform(1:3,1:3) = rotationMatrix(0,0,0);
    tform(1,4)=-10;%x value
    tform(2,4)=-4.7;%y value
    tform=affine3d(tform');
    pointCloudResult3 = pctransform(pointCloudResult1,tform);


    hold on
    pcshowpair(pointCloudResult2, pointCloudResult1)
    %pcshow(pointCloudResult3)
    grid on
    box on
    view(90,-90)
    set(gca,'color','w', 'XColor','k', 'YColor','k');
    set(gcf,'color','w');
    
    xlabel('Y-Axis in m', 'Interpreter', 'latex')
    ylabel('X-Axis in m', 'Interpreter', 'latex')
    % title(num2str(i), 'Interpreter', 'latex')
    pbaspect([1 1.0 1])

    %axis equal

    saveas(gcf, 'outputPDFs/matchedPointclouds', 'pdf')
    system('pdfcrop outputPDFs/matchedPointclouds.pdf outputPDFs/matchedPointclouds.pdf');
    


    figure(11)
    clf

    correlationMatrixShift1D = readmatrix(['csvFiles/resultingCorrelationShift' num2str(indexInitialGuess-1) '.csv']);
    resultSize = nthroot(length(correlationMatrixShift1D),2);
    %A = reshape(results,resultSize,resultSize);
    correlationMatrixShift2D = reshape(correlationMatrixShift1D,resultSize,resultSize);
    imagesc(correlationMatrixShift2D);
    [Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
    %surf(Xplot,Yplot,(correlationMatrixShift2D),'edgecolor', 'none');
    %xlabel("x-axis", 'Interpreter', 'latex');
    %ylabel("y-axis", 'Interpreter', 'latex');
    
    %title('Correlation Surface of  Best Performing Match', 'Interpreter', 'latex')
    box on 
    %title(num2str(i), 'Interpreter', 'latex')
    pbaspect([1 1 1])
    
    %set(gcf,'units','points','position',[0,0,500,500])
    
    
    saveas(gcf, 'outputPDFs/2dCorrelation', 'pdf')
    system('pdfcrop outputPDFs/2dCorrelation.pdf outputPDFs/2dCorrelation.pdf');



end




