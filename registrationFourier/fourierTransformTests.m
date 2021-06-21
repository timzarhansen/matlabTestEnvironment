clc
clear

pointCloud = pcread("after_voxel_second.pcd");


%% create voxel grid of pcl %%
fromTo = 30;
numberOfPoints=240;%120;
voxelData1 = zeros(numberOfPoints,numberOfPoints);

for j=1:pointCloud.Count
    xPos=pointCloud.Location(j,1);
    yPos=pointCloud.Location(j,2);
    xIndex = cast((xPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    yIndex = cast((yPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    voxelData1(xIndex,yIndex) = 1;
end

[spectrum1,magnitude1,phase1] = plotffts(voxelData1,1);
%% create voxel grid of pcl and shift by value%%
voxelData2 = zeros(numberOfPoints,numberOfPoints);
shiftfirst = [-10,-5];
for j=1:pointCloud.Count
    xPos=pointCloud.Location(j,1)+shiftfirst(1);
    yPos=pointCloud.Location(j,2)+shiftfirst(2);
    xIndex = cast((xPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    yIndex = cast((yPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    voxelData2(xIndex,yIndex) = 1;
end

% add two copies of this point Cloud
shiftSecond = [-10,10];
for j=1:pointCloud.Count
    xPos=pointCloud.Location(j,1)+shiftSecond(1);
    yPos=pointCloud.Location(j,2)+shiftSecond(2);
    xIndex = cast((xPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    yIndex = cast((yPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    voxelData2(xIndex,yIndex) = 1;
end


voxelData2=voxelData2 + 0.1*randn(size(voxelData2));
[spectrum2,magnitude2,phase2] =plotffts(voxelData2,2);




%% calculate translation

resultingphaseDifference = phase1 - phase2;

inverseFFTForPhase = fftshift(ifft2(exp(i*resultingphaseDifference)));

immaginaryPart=imag(inverseFFTForPhase);
realPart = real(inverseFFTForPhase);
phase=atan2(immaginaryPart, realPart);



%figure(3)
%imagesc(phase);
%axis image

magnitude = abs(inverseFFTForPhase);
figure(4)
%imagesc(magnitude);
[Xplot,Yplot]=meshgrid(1:numberOfPoints,1:numberOfPoints);
surf(Xplot,Yplot,magnitude)
title('magnitude of invFFT(arg(R)-arg(S)) ')
%axis equal

%%

[TF1,P] = islocalmax(magnitude);

figure(4)
[Xplot,Yplot]=meshgrid(1:numberOfPoints,1:numberOfPoints);
surf(Xplot,Yplot,magnitude)
%imagesc(P);
title('magnitude of invFFT(arg(R)-arg(S)) ')
%axis image

Preshaped = reshape(P,1,[]);

[peaksOfShift,I] = sort(Preshaped,'descend');

diffPeaks = abs(diff(peaksOfShift));
[TF1,pos] = max(diffPeaks);

for j=1:pos
    indexPCol(j) = ceil(I(j)/numberOfPoints);
    indexPRow(j) = I(j)-numberOfPoints*(indexPCol(j)-1);
    position(j,1:2) = [indexPRow(j)-numberOfPoints/2,indexPCol(j)-numberOfPoints/2];
end


%%

translation_shift1 = -position(1,:)/numberOfPoints*fromTo*2
translation_shift2 = -position(2,:)/numberOfPoints*fromTo*2

shiftSecond
shiftfirst















