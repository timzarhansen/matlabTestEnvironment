clc
clear
number = 1;

voxelData1Raw = readmatrix("dataFolder/testFolder/correctedScan"+number+".csv");
voxelData2Raw = readmatrix("dataFolder/testFolder/uncorrectedScan"+number+".csv");




figure(1)
imagesc(voxelData1Raw)
% box on
axis image

figure(2)
imagesc(voxelData2Raw)
% box on
axis image


%%
clc
clear
N=200;

inputExample = zeros(N,N);



p=98:102;
k=5;
inputExample(p,p) = ones(k,k);



resultFFT = fftshift(fft2(inputExample,N,N));

imagesc(angle(resultFFT))
axis equal





