clc
clear

%pointCloud = pcread('teapot.ply');
pointCloud = pcread("after_voxel_second.pcd");

% create voxel grid of pcl %%
fromTo = 30;
numberOfPoints=32;


shiftfirst = [0,0,0];
rotation = rotationMatrix(0.0,0.0,0.0);

voxelData1 = getVoxelData(numberOfPoints,shiftfirst,rotation,pointCloud,fromTo);

%voxelData1(:,:,size(voxelData1,1)/2) = imgaussfilt(voxelData1(:,:,size(voxelData1,1)/2),4);
% 3D:
[spectrum1,magnitude1,phase1] = plotffts(voxelData1,1);

% create voxel grid of pcl and shift by value%%

shiftfirst = [0,0,0];
rotation = rotationMatrix(0.0,0.0,0.5);

voxelData2 =getVoxelData(numberOfPoints,shiftfirst,rotation,pointCloud,fromTo);
%voxelData2(:,:,size(voxelData2,1)/2) = imgaussfilt(voxelData2(:,:,size(voxelData2,1)/2),4);

% 3D:
voxelData2=voxelData2 + 0.0*randn(size(voxelData2));
[spectrum2,magnitude2,phase2] =plotffts(voxelData2,2);

% calculate sampled f(theta,phi)

rNumbers=4:numberOfPoints/2-2;%2:31;%was 4:62
B=numberOfPoints/2;

thetaIndex = 1:2*B;
phiIndex = 1:2*B;
phi = pi*phiIndex/B;
theta = pi*(2*thetaIndex+1)/(4*B);


maximumSet2 = max(magnitude1, [], "all", "linear");
magnitude1=magnitude1/maximumSet2;
magnitude2=magnitude2/maximumSet2;


% 2D:
fThetaPhi1 = sampledFThetaPhi(magnitude1,theta,phi,B,rNumbers);
fThetaPhi2 = sampledFThetaPhi(magnitude2,theta,phi,B,rNumbers);

% if 1
%     figure(3)
%     sphere(1000);
%     ch = get(gca,'children');
%     set(ch,'facecolor','texturemap','cdata',fThetaPhi1,'edgecolor','none');
%     axis equal
%     
%     
%     figure(4)
%     
%     sphere(1000);
%     ch = get(gca,'children');
%     set(ch,'facecolor','texturemap','cdata',fThetaPhi2,'edgecolor','none');
%     axis equal
% else
%     figure(3)
%     imagesc((fThetaPhi1));
%     axis image
%     
%     
%     figure(4)
%     imagesc((fThetaPhi2));
%     axis image
% 
% end



fThetaPhi1Interleaved = matrix2InterleavedFormat(fThetaPhi1);
fThetaPhi2Interleaved = matrix2InterleavedFormat(fThetaPhi2);

writematrix(fThetaPhi1Interleaved',"matrixone.csv")
writematrix(fThetaPhi2Interleaved',"matrixtwo.csv")

%%

% figure(7)
% yawArray=linspace(-fromTo,fromTo,numberOfPoints);
% rollArray=linspace(-fromTo,fromTo,numberOfPoints);
% pitchArray=linspace(-fromTo,fromTo,numberOfPoints);
% [mg3dX, mg3dY, mg3dZ] = meshgrid(yawArray,rollArray,pitchArray);
% magnitude2TMP = 1-magnitude2(:)/max(magnitude2(:));
% 
% scatterPlot = scatter3(mg3dX(:),mg3dY(:),mg3dZ(:),8,[1*magnitude2TMP,1*magnitude2TMP,1*magnitude2TMP],'filled');
% alpha = 0.9;
% set(scatterPlot, 'MarkerEdgeAlpha', alpha, 'MarkerFaceAlpha', alpha)
% 
% axis equal
% 
% figure(8)
% magnitude1TMP = 1-magnitude1(:)/max(magnitude1(:));
% 
% scatterPlot2 = scatter3(mg3dX(:),mg3dY(:),mg3dZ(:),8,[1*magnitude1TMP,1*magnitude1TMP,1*magnitude1TMP],'filled');
% 
% set(scatterPlot2, 'MarkerEdgeAlpha', alpha, 'MarkerFaceAlpha', alpha)
% 
% axis equal

%%
N=numberOfPoints;
results = readmatrix("ergWrap.txt");

results = results/max(results);

A = reshape(results,N,N,N);
%


% x = linspace(0,2*pi,256);
% y = linspace(0,2*pi,256);
% [X,Y] = meshgrid(x,y);
% for i = 1:256
%     surf(X,Y,squeeze(A(:,:,i)),'edgecolor', 'none');
%     xlim([0 7]);
%     ylim([0 7]);
%     zlim([0.94 1]);
%     pause(0.01)
% end

%contourf(X,Y,squeeze(A(:,:,i)));
%

correlationNumberMatrix = squeeze(A(:,:,1));

thetaList = linspace(0,2*pi,N);
psiList = linspace(0,2*pi,N);
listOfCorrelationAndAngle = zeros(N*N,2);
for j=1:N
    for k=1:N
        currentAngle = mod(thetaList(j)+psiList(k)+4*pi,2*pi);
        listOfCorrelationAndAngle(j*N-N+k,1)=currentAngle;
        listOfCorrelationAndAngle(j*N-N+k,2)=correlationNumberMatrix(k,j);
    end
end
figure(3)
plot(listOfCorrelationAndAngle(:,1),listOfCorrelationAndAngle(:,2),".")
%

listOfCorrelationAndAngle=sortrows(listOfCorrelationAndAngle);
currentaverageAngle = listOfCorrelationAndAngle(1,1);
numberOfAngles=1;
averageCorrelation = listOfCorrelationAndAngle(1,2);
i=1;
for k = 2:length(listOfCorrelationAndAngle)
    if abs(currentaverageAngle-listOfCorrelationAndAngle(k,1))<1/N/4
        numberOfAngles = numberOfAngles+1;
        averageCorrelation = averageCorrelation+listOfCorrelationAndAngle(k,2);
    else
        resultingPlotting(i,:)=[currentaverageAngle,averageCorrelation/numberOfAngles];

        i=i+1;
        numberOfAngles=1;
        currentaverageAngle = listOfCorrelationAndAngle(k,1);
        averageCorrelation = listOfCorrelationAndAngle(k,2);
    end

end
resultingPlotting(i,:)=[currentaverageAngle,averageCorrelation/numberOfAngles];
%
plot(resultingPlotting(:,1),resultingPlotting(:,2),".")
[pks,potentialRotations] = findpeaks(smooth(resultingPlotting(:,2)),resultingPlotting(:,1));
%potentialRotations = 2*pi-potentialRotations;
display(potentialRotations)
%% show rotations

% for potRotIndex = 1:length(potentialRotations)
%     figure(6)
%     imagesc(voxelData1(:,:,size(voxelData1,1)/2))
%     figure(7)
%     imagesc(imrotate(voxelData2(:,:,size(voxelData2,1)/2),potentialRotations(potRotIndex)/pi*180,'bilinear','crop'));
%     pause(1)
% end



%% translation correlation for one example 
voxelData2Rotated = voxelData2;
voxelData2Rotated(:,:,size(voxelData2,1)/2) = imrotate(voxelData2(:,:,size(voxelData2,1)/2),0.5/pi*180,'bilinear','crop');

[spectrum2Rotated,magnitude2Rotated,phase2Rotated] = plotffts(voxelData2Rotated,4);


resultingphaseDifference = phase1 - phase2Rotated;

inverseFFTForPhase = fftshift(ifftn(exp(complex(zeros(N,N,N),resultingphaseDifference))));

imaginaryPart=imag(inverseFFTForPhase);
realPart = real(inverseFFTForPhase);
phase=atan2(imaginaryPart, realPart);


magnitude = abs(inverseFFTForPhase);
magnitude  = magnitude(:,:,size(voxelData2,1)/2+1);
figure(5)
%imagesc(magnitude);
[Xplot,Yplot]=meshgrid(1:numberOfPoints,1:numberOfPoints);
surf(Xplot,Yplot,(magnitude),'edgecolor', 'none');
title('magnitude of invFFT(arg(R)-arg(S)) ')
%zlim([0,0.15])
%
[M,I] = max(abs(inverseFFTForPhase), [], "all", "linear");
[dim1, dim2, dim3] = ind2sub(size(A),I);
%snr(magnitude(:))

%

[TF1,P] = islocalmax(magnitude);

Preshaped = reshape(P,1,[]);

[peaksOfShift,I] = sort(Preshaped,'descend');

diffPeaks = abs(diff(peaksOfShift));
[TF1,pos] = max(diffPeaks)

for j=1:pos
    indexPCol(j) = ceil(I(j)/numberOfPoints);
    indexPRow(j) = I(j)-numberOfPoints*(indexPCol(j)-1);
    position(j,1:2) = [indexPRow(j)-numberOfPoints/2,indexPCol(j)-numberOfPoints/2];
end

