clc
clear
data = readJsonGraphPCL("outputSlam.json");


%% plot two pcl
positionCloudOne = 18;
positionCloudZwo = positionCloudOne+1;

pointcloud1 = squeeze(data(positionCloudOne,:,:));
pointcloud1(pointcloud1(:, 1)== 0 & pointcloud1(:, 2)== 0 & pointcloud1(:, 3)== 0, :)= [];


pointcloud2 = squeeze(data(positionCloudZwo,:,:));
pointcloud2(pointcloud2(:, 1)== 0 & pointcloud2(:, 2)== 0 & pointcloud2(:, 3)== 0, :)= [];

%% create voxel grid of pcl %%
fromTo = 60;
numberOfPoints=64;%120;
voxelData1 = zeros(numberOfPoints,numberOfPoints,numberOfPoints);

for j=1:size(pointcloud1,1)
    xPos=pointcloud1(j,1);
    yPos=pointcloud1(j,2);
    xIndex = cast((xPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    yIndex = cast((yPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    zIndex = cast((0 + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    voxelData1(xIndex,yIndex,zIndex) = 1;
end

[spectrum1,magnitude1,phase1] = plotffts(voxelData1,1);


voxelData2 = zeros(numberOfPoints,numberOfPoints,numberOfPoints);

for j=1:size(pointcloud2,1)
    xPos=pointcloud2(j,1);
    yPos=pointcloud2(j,2);
    xIndex = cast((xPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    yIndex = cast((yPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    zIndex = cast((0 + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    voxelData2(xIndex,yIndex,zIndex) = 1;
end

[spectrum2,magnitude2,phase2] = plotffts(voxelData2,2);



%% calculate rotation

rNumbers=2:31;%was 4:62
B=numberOfPoints/2;

thetaIndex = 1:2*B;
phiIndex = 1:2*B;
phi = pi*phiIndex/B;
theta = pi*(2*thetaIndex+1)/(4*B);





% 2D:
fThetaPhi1 = sampledFThetaPhi(magnitude1,theta,phi,B,rNumbers);
fThetaPhi2 = sampledFThetaPhi(magnitude2,theta,phi,B,rNumbers);

figure(3)
imagesc((fThetaPhi1));
axis image


figure(4)
imagesc((fThetaPhi2));
axis image


% calculate fourie coeff of fThetaPhi1
flm1 = fourieCoeff(fThetaPhi1,numberOfPoints,B,theta,phi);

% calculate fourie coeff of fThetaPhi2
flm2 = fourieCoeff(fThetaPhi2,numberOfPoints,B,theta,phi);
%%

NTest = 40;
yawTMP=linspace(-pi,pi,NTest);
COutput = zeros(NTest,1);
for j = 1:NTest
    COutput(j) = calculateScoreOfRotation(yawTMP(j),flm1,flm2,B);
end


%% calculate translation

resultingphaseDifference = phase1 - phase2;

inverseFFTForPhase = fftshift(ifft2(exp(i*resultingphaseDifference)));

immaginaryPart=imag(inverseFFTForPhase);
realPart = real(inverseFFTForPhase);
phase=atan2(immaginaryPart, realPart);




%%
magnitude = abs(inverseFFTForPhase);
magnitude  = magnitude(:,:,size(magnitude,1)/2);
magnitude = imgaussfilt(magnitude,2);

figure(5)
imagesc(magnitude);
axis image

%%

[P] = imregionalmax(magnitude);

figure(6)
[Xplot,Yplot]=meshgrid(1:numberOfPoints,1:numberOfPoints);

surf(Xplot,Yplot,magnitude)
%imagesc(P);
title('magnitude of invFFT(arg(R)-arg(S)) ')
%axis image
%%
Preshaped = reshape(magnitude.*P,1,[]);

[peaksOfShift,I] = sort(Preshaped,'descend');

diffPeaks = abs(diff(peaksOfShift));
[TF1,pos] = max(diffPeaks);
%pos = pos+1
for j=1:pos
    indexPCol(j) = ceil(I(j)/numberOfPoints);
    indexPRow(j) = I(j)-numberOfPoints*(indexPCol(j)-1);
    position(j,1:2) = [indexPRow(j)-numberOfPoints/2,indexPCol(j)-numberOfPoints/2];
end


%%
for j=1:pos
    translationTMP = -position(j,:)/numberOfPoints*fromTo*2
    translation_shift(j,1:2) = translationTMP;
end



%% test -1.55 first value 1.65 second value
figure(7)
from = 20;
to = 40;
tmp = abs(COutput)/(10^12);
gaussEqn = 'a*exp(-((x-b)/c)^2)+d';
startValues = [0.1 0 1 1];
f = fit(yawTMP(from:to)',tmp(from:to),gaussEqn,'Start',startValues);

plot(f,yawTMP(from:to),tmp(from:to))
