clc
clear


magnitudeFFTW1 = readmatrix("csvFiles/magnitudeFFTW1.csv");
N=sqrt(size(magnitudeFFTW1,1));

phaseFFTW1 = readmatrix("csvFiles/phaseFFTW1.csv");
voxelDataUsed1 = readmatrix("csvFiles/voxelDataFFTW1.csv");

magnitudeFFTW2 = readmatrix("csvFiles/magnitudeFFTW2.csv");
phaseFFTW2 = readmatrix("csvFiles/phaseFFTW2.csv");
voxelDataUsed2 = readmatrix("csvFiles/voxelDataFFTW2.csv");

magnitude1 =zeros(N,N);
phase1 =zeros(N,N);
voxelData1 =zeros(N,N);
magnitude2 =zeros(N,N);
phase2 =zeros(N,N);
voxelData2 =zeros(N,N);
for j =1:N
    for i =1:N
        magnitude1(i,j) = magnitudeFFTW1((i-1)*N+j);
        phase1(i,j) = phaseFFTW1((i-1)*N+j);
        voxelData1(i,j) = voxelDataUsed1((i-1)*N+j);
        magnitude2(i,j) = magnitudeFFTW2((i-1)*N+j);
        phase2(i,j) = phaseFFTW2((i-1)*N+j);
        voxelData2(i,j) = voxelDataUsed2((i-1)*N+j);
    end
end


% magnitude1=fftshift(magnitude1);
% phase1=fftshift(phase1);
% magnitude2=fftshift(magnitude2);
% phase2=fftshift(phase2);

figure(1)
clf
subplot( 1, 2, 2 )

% imagesc(squeeze(hipass_filter(size(magnitude1, 1),size(magnitude1,2)).*magnitude1));
imagesc(squeeze(magnitude1));


%imshow(magnitude2(:,:,size(magnitude2,1)/2));
title('Magnitude Voxel: '+string(1))
axis image

% subplot( 1, 3, 3 )
% imagesc(phase1);
% %imagesc(squeeze(phase(size(phase,1)/2,:,:)));
% title('Phase Voxel: '+string(1))
% axis image


subplot( 1, 2, 1 )

%N = nthroot(length(voxelDataUsed),3);

imagesc((voxelData1))
axis image
% figure 2
figure(2)
clf
subplot( 1, 2, 2 )

% imagesc(squeeze(hipass_filter(size(magnitude2, 1),size(magnitude2,2)).*magnitude2));
imagesc(squeeze(magnitude2));

%imshow(magnitude2(:,:,size(magnitude2,1)/2));
title('Magnitude Voxel: '+string(1))
axis image

% subplot( 1, 3, 3 )
% imagesc(squeeze(phase2));
% %imagesc(squeeze(phase(size(phase,1)/2,:,:)));
% title('Phase Voxel: '+string(1))
% axis image


subplot( 1, 2, 1 )

%N = nthroot(length(voxelDataUsed),3);

imagesc((voxelData2))
axis image
%%
% resampledDataForSphere1 = readmatrix("csvFiles/resampledVoxel1.csv");
% resampledDataForSphere2 = readmatrix("csvFiles/resampledVoxel2.csv");
% 
% resampledDataForSphereResult1 =zeros(N,N);
% resampledDataForSphereResult2 =zeros(N,N);
% for j = 1:N
%     for i =1:N
%             resampledDataForSphereResult1(j,i) = resampledDataForSphere1((i-1)*N+j);
%             resampledDataForSphereResult2(j,i) = resampledDataForSphere2((i-1)*N+j);
%     end
% end

% currently not that interesting
% if 1
%     figure(4)
% 
%     sphere(1000);
%     ch = get(gca,'children');
%     set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult2,'edgecolor','none');
%     axis equal
% 
%     figure(5)
% 
%     sphere(1000);
%     ch = get(gca,'children');
%     set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
%     axis equal
% 
% else
%     figure(4)
%     imagesc((resampledDataForSphereResult1));
%     axis image
%     figure(5)
%     imagesc((resampledDataForSphereResult2));
%     axis image
%     
%     
% end


%% read in result
% N=256;
results = readmatrix("csvFiles/resultCorrelation3D.csv");

results = results/max(results);
resultSize = nthroot(length(results),3);

A = reshape(results,resultSize,resultSize,resultSize);

%%
correlationNumberMatrix = squeeze(A(:,:,1));
% figure(9)
% imagesc(correlationNumberMatrix)

%%

% thetaList = linspace(0,2*pi-2*pi/resultSize,resultSize);
% psiList = linspace(0,2*pi-2*pi/resultSize,resultSize);
% listOfCorrelationAndAngle = zeros(resultSize*resultSize,2);
% for j=1:resultSize
%     for k=1:resultSize
%         currentAngle = mod((thetaList(k)+psiList(j))+6*pi,2*pi);
%         listOfCorrelationAndAngle(k*resultSize-resultSize+j,1)=currentAngle;
%         listOfCorrelationAndAngle(k*resultSize-resultSize+j,2)=correlationNumberMatrix(k,j);
%     end
% end
% figure(5)
% 
% plot(listOfCorrelationAndAngle(:,1),listOfCorrelationAndAngle(:,2),".")
%%






% listOfCorrelationAndAngle=sortrows(listOfCorrelationAndAngle);
% currentaverageAngle = listOfCorrelationAndAngle(1,1);
% numberOfAngles=1;
% averageCorrelation = listOfCorrelationAndAngle(1,2);
% i=1;
% for k = 2:length(listOfCorrelationAndAngle)
%     if abs(currentaverageAngle-listOfCorrelationAndAngle(k,1))<1/N/4
%         numberOfAngles = numberOfAngles+1;
%         averageCorrelation = averageCorrelation+listOfCorrelationAndAngle(k,2);
%     else
%         resultingPlotting(i,:)=[currentaverageAngle,averageCorrelation/numberOfAngles];
% 
%         i=i+1;
%         numberOfAngles=1;
%         currentaverageAngle = listOfCorrelationAndAngle(k,1);
%         averageCorrelation = listOfCorrelationAndAngle(k,2);
%     end
% 
% end
% resultingPlotting(i,:)=[currentaverageAngle,averageCorrelation/numberOfAngles];
% 
% figure(9)
% clf
% plot(resultingPlotting(:,1),resultingPlotting(:,2),".")
% [pks,potentialRotations] = findpeaks(smooth(resultingPlotting(:,2)),resultingPlotting(:,1));

%potentialRotations = 2*pi-potentialRotations;
% display(potentialRotations)

% hold on
% currentPeakXValue = potentialRotations(1);
% 
% y = resultingPlotting(find(resultingPlotting(:,1)==currentPeakXValue)-5:1:find(resultingPlotting(:,1)==currentPeakXValue)+5,2);
% x = resultingPlotting(find(resultingPlotting(:,1)==currentPeakXValue)-5:1:find(resultingPlotting(:,1)==currentPeakXValue)+5,1);
% 
% f = fit(x,y,"gauss1")
% plot(f,x,y)


%%
figure(6)
clf
correlationOfAngles = readmatrix("csvFiles/resultingCorrelation1D.csv");

xForPlot = 0:size(correlationOfAngles,1)-1;
xForPlot = xForPlot/size(correlationOfAngles,1)*360;
plot(xForPlot,correlationOfAngles)
%%
figure(8)
clf

correlationMatrixShift1D = readmatrix("csvFiles/resultingCorrelationShift.csv");
resultSize = nthroot(length(correlationMatrixShift1D),2);
%A = reshape(results,resultSize,resultSize);
correlationMatrixShift2D =zeros(resultSize);
for j = 1:resultSize
    for i =1:resultSize
            correlationMatrixShift2D(i,j) = correlationMatrixShift1D((i-1)*resultSize+j);
            correlationMatrixShift2D(i,j) = correlationMatrixShift1D((i-1)*resultSize+j);
    end
end

%correlationMatrixShift2D = reshape(correlationMatrixShift1D,resultSize,resultSize);


% [M,I] = max(correlationMatrixShift2D, [], "all", "linear");
% [dim1, dim2, dim3] = ind2sub(size(correlationMatrixShift2D),I)

%correlationMatrixShift2D  = squeeze(correlationMatrixShift3D(65,:,:));

%imagesc(magnitude);
testImage = correlationMatrixShift2D;

% for i=1:50
%     testImage=imgaussfilt(testImage,'FilterSize',5);
% end




[Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
surf(Xplot,Yplot,(testImage),'edgecolor', 'none');
xlabel("x-axis");
ylabel("y-axis");


%% test of own translation Convolution

voxelData1TransPadding = voxelData1;
voxelData2TransPadding = (((voxelData2)));
% for i=1:10
%     voxelData1TransPadding=imgaussfilt(voxelData1Trans,'FilterSize',5);
%     voxelData2TransPadding=imgaussfilt(voxelData2Trans,'FilterSize',5);
% end

paddingSize = 0;
voxelData1TransPadding=padarray(voxelData1TransPadding,[paddingSize paddingSize],0,'both');
voxelData2TransPadding=padarray(voxelData2TransPadding,[paddingSize paddingSize],0,'both');


% [spectrum1Trans,magnitude1Trans,phase1Trans] = plotffts2D(voxelData1TransPadding,1);
% [spectrum2Trans,magnitude2Trans,phase2Trans] = plotffts2D(voxelData2TransPadding,2);

spectrum1Trans = (fftn(voxelData1TransPadding));
spectrum2Trans = (fftn(((voxelData2TransPadding))));

%resultingphaseDifference = phase1Trans - phase2Trans;

%inverseFFTForPhase = fftshift(ifft2(exp(complex(0,resultingphaseDifference))));
%inverseFFTForPhase = fftshift(ifft2(spectrum1Trans.*conj(spectrum2Trans))./(abs(spectrum1Trans.*conj(spectrum2Trans))));

inverseFFTForPhase = fftshift(ifft2(128*128*spectrum1Trans.*conj(spectrum2Trans)));

magnitudeTranslation = abs(inverseFFTForPhase);


% for i=1:20
%     magnitudeTranslation=imgaussfilt(magnitudeTranslation,'FilterSize',5);
% end

% figure(9)
% %imagesc(magnitude);
% 
% [Xplot,Yplot]=meshgrid(1:size(magnitudeTranslation,1),1:size(magnitudeTranslation,1));
% %surf(Xplot,Yplot,(magnitudeTranslation),'edgecolor', 'none')
% surf(Xplot,Yplot,(magnitudeTranslation))
% 
% 
% title('Convolution ')
% 
% 
% xlabel("x-axis");
% ylabel("y-axis");

%% after convolution calculate gauss

[M,I] = max(magnitudeTranslation,[],"all","linear");
[yAxisIndex, xAxisIndex] = ind2sub(size(magnitudeTranslation),I);

howManyRemove = 0;
%y = squeeze(magnitudeTranslation(:,xAxisIndex));
%shiftXY = (yAxisIndex-1)*0.6;

y = squeeze(magnitudeTranslation(yAxisIndex,:))';
shiftXY = (xAxisIndex-1)*0.6;
x = linspace(0,0.6*(N-1),N);


% figure(5)
% clf




% plot(x,y);
% 
% y = [y(howManyRemove+1:end-howManyRemove)];
% x = [x(howManyRemove+1:end-howManyRemove)];
% 
% 
% hold on 
% 
% gaussEqn = string(M)+'*exp(-((x-'+string(shiftXY)+')/c/2)^2)';
% f = fit(x',y,gaussEqn,'StartPoint', [20])
% plot(f,x,y);



%% Correlation
% voxelData1Trans = voxelData1;
% voxelData2Trans = voxelData2;
% % for i=1:10
% %     voxelData1Trans=imgaussfilt(voxelData1Trans,'FilterSize',5);
% %     voxelData2Trans=imgaussfilt(voxelData2Trans,'FilterSize',5);
% % end
% % for i=1:20
% %     magnitudeTranslation=imgaussfilt(magnitudeTranslation,'FilterSize',5);
% % end
% correlationImage = xcorr2(voxelData1Trans,voxelData2Trans);
% figure(10)
% [Xplot,Yplot]=meshgrid(1:size(correlationImage,1),1:size(correlationImage,1));
% %surf(Xplot,Yplot,correlationImage,'edgecolor', 'none')
% surf(Xplot,Yplot,correlationImage)
% 
% title('Correlation ')
% 
% xlabel("x-axis");
% ylabel("y-axis");


%% calc spectrum

% spectrum1 = (magnitude1.*exp(complex(0,phase1)));
%spectrum2 = magnitude1*exp(complex(0,phase1));

% spectrum1ByVoxel = fftshift(fftn(voxelData1));
% spectrum2ByVoxel = fftn(voxelData2TransPadding);
% [spectrum1ByVoxel,magnitude1ByVoxel,phase1ByVoxel] = plotffts2D(voxelData1,2);
% TestSpectrum=magnitude1ByVoxel.*exp(complex(0,phase1ByVoxel));



%% show resulting PCLs

figure(3)
clf
voxelResult1FFTW = readmatrix("csvFiles/resultVoxel1.csv");
N=sqrt(size(voxelResult1FFTW,1));

voxelResult2FFTW = readmatrix("csvFiles/resultVoxel2.csv");


voxelResult1 =zeros(N,N);
voxelResult2 =zeros(N,N);
for j =1:N
    for k =1:N
        voxelResult1(k,j) = voxelResult1FFTW(k*N-N+j);
        voxelResult2(k,j) = voxelResult2FFTW(k*N-N+j);

    end
end


%voxelResult1=fftshift(voxelResult1);
%voxelResult2=fftshift(voxelResult2);






subplot( 1, 2, 1 )

imagesc(squeeze(voxelResult1));


%imshow(magnitude2(:,:,size(magnitude2,1)/2));
title('Voxel 1: ')
axis image

subplot( 1, 2, 2 )
%imagesc(squeeze(voxelResult2));
C = imfuse(voxelResult1,voxelResult2,'blend','Scaling','joint');
imagesc(squeeze(C));
%imagesc(squeeze(phase(size(phase,1)/2,:,:)));
title('Voxel 2: ')
axis image






% pointCloudResult1 = pcread("resultingPCL1.pcd");
% pointCloudResult2 = pcread("resultingPCL2.pcd");
% tform = eye(4);
% tform(1:3,1:3)=rotationMatrix(0,0,0);
% tform(1,4)=0;%x value
% tform(2,4)=0;%y value
% tform=affine3d(tform');
% pointCloudResult1 = pctransform(pointCloudResult1,tform);
% 
% 
% 
% pcshowpair(pointCloudResult1, pointCloudResult2)
%% cross corelation

% voxelData1TMP = voxelData1(size(voxelData1,1)/4:3*size(voxelData1,1)/4,size(voxelData1,1)/4:3*size(voxelData1,1)/4);
% voxelData2TMP = voxelData2(size(voxelData1,1)/4:3*size(voxelData1,1)/4,size(voxelData1,1)/4:3*size(voxelData1,1)/4);
% 
% resultingCrossCorrelation = xcorr2(voxelData1TMP,voxelData2TMP);
% 
% [Xplot,Yplot] = meshgrid(1:size(resultingCrossCorrelation,1),1:size(resultingCrossCorrelation,1));
% surf(Xplot,Yplot,((resultingCrossCorrelation)),'EdgeColor','none');
% xlabel("x-axis");
% ylabel("y-axis");

figure(9)
A_FFT = (fft2(voxelData1));
imagesc(abs(A_FFT))

B_FFT = (fft2(voxelData2));


C_New = fftshift(ifft2(A_FFT.*conj(B_FFT)));


[Xplot,Yplot] = meshgrid(1:size(C_New,1),1:size(C_New,1));
surf(Xplot,Yplot,(C_New),'EdgeColor','none');
xlabel("x-axis");
ylabel("y-axis");




