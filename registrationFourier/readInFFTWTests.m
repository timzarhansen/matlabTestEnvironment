clc
clear

fromTo = 30;
N=128;
magnitudeFFTW1 = readmatrix("magnitudeFFTW1.csv");
phaseFFTW1 = readmatrix("phaseFFTW1.csv");
voxelDataUsed1 = readmatrix("voxelDataFFTW1.csv");

magnitudeFFTW2 = readmatrix("magnitudeFFTW2.csv");
phaseFFTW2 = readmatrix("phaseFFTW2.csv");
voxelDataUsed2 = readmatrix("voxelDataFFTW2.csv");
%fftwResult = complex(realFFTW,imaginaryFFTW);
%fftwResult = (reshape(fftwResult,N,N,N));
magnitude1 =zeros(N,N,N);
phase1 =zeros(N,N,N);
voxelData1 =zeros(N,N,N);
magnitude2 =zeros(N,N,N);
phase2 =zeros(N,N,N);
voxelData2 =zeros(N,N,N);
for i = 1:N
    for j =1:N
        for k =1:N
            magnitude1(i,j,k) = magnitudeFFTW1(i*N*N-N*N+j*N-N+k);
            phase1(i,j,k) = phaseFFTW1(i*N*N-N*N+j*N-N+k);
            voxelData1(i,j,k) = voxelDataUsed1((i-1)*N*N+(j-1)*N+k);
            magnitude2(i,j,k) = magnitudeFFTW2(i*N*N-N*N+j*N-N+k);
            phase2(i,j,k) = phaseFFTW2(i*N*N-N*N+j*N-N+k);
            voxelData2(i,j,k) = voxelDataUsed2((i-1)*N*N+(j-1)*N+k);
        end
    end
end

magnitude1=fftshift(magnitude1);
phase1=fftshift(phase1);
magnitude2=fftshift(magnitude2);
phase2=fftshift(phase2);

figure(1)
subplot( 1, 3, 2 )

imagesc(squeeze(magnitude1(:,:,size(magnitude1,1)/2)));


%imshow(magnitude2(:,:,size(magnitude2,1)/2));
title('Magnitude Voxel: '+string(1))
axis image

subplot( 1, 3, 3 )
imagesc(squeeze(phase1(:,:,size(phase1,1)/2)));
%imagesc(squeeze(phase(size(phase,1)/2,:,:)));
title('Phase Voxel: '+string(1))
axis image


subplot( 1, 3, 1 )

%N = nthroot(length(voxelDataUsed),3);

imagesc((voxelData1(:,:,size(voxelData1,1)/2)))
axis image
% figure 2
figure(2)
subplot( 1, 3, 2 )

imagesc(squeeze(magnitude2(:,:,size(magnitude2,1)/2)));


%imshow(magnitude2(:,:,size(magnitude2,1)/2));
title('Magnitude Voxel: '+string(1))
axis image

subplot( 1, 3, 3 )
imagesc(squeeze(phase2(:,:,size(phase2,1)/2)));
%imagesc(squeeze(phase(size(phase,1)/2,:,:)));
title('Phase Voxel: '+string(1))
axis image


subplot( 1, 3, 1 )

%N = nthroot(length(voxelDataUsed),3);

imagesc((voxelData2(:,:,size(voxelData2,1)/2)))
axis image
%%
resampledDataForSphere1 = readmatrix("resampledVoxel1.csv");
resampledDataForSphere2 = readmatrix("resampledVoxel2.csv");

resampledDataForSphereResult1 =zeros(N,N);
resampledDataForSphereResult2 =zeros(N,N);
for i = 1:N
    for j =1:N
            resampledDataForSphereResult1(i,j) = resampledDataForSphere1((i-1)*N+j);
            resampledDataForSphereResult2(i,j) = resampledDataForSphere2((i-1)*N+j);
    end
end

if 1
    figure(3)
    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult2,'edgecolor','none');
    axis equal

    figure(4)
    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
    axis equal

else
    figure(3)
    imagesc((fThetaPhi1));
    axis image
    figure(4)
    imagesc((resampledDataForSphereResult));
    axis image
    
    
end


%% read in result
N=128;
results = readmatrix("resultCorrelation.csv");

results = results/max(results);
resultSize = nthroot(length(results),3);
A = reshape(results,resultSize,resultSize,resultSize);
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

thetaList = linspace(0,2*pi,resultSize);
psiList = linspace(0,2*pi,resultSize);
listOfCorrelationAndAngle = zeros(resultSize*resultSize,2);
for j=1:resultSize
    for k=1:resultSize
        currentAngle = mod(thetaList(j)+psiList(k)+4*pi,2*pi);
        listOfCorrelationAndAngle(j*resultSize-resultSize+k,1)=currentAngle;
        listOfCorrelationAndAngle(j*resultSize-resultSize+k,2)=correlationNumberMatrix(k,j);
    end
end
figure(5)
clf
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
figure(6)
clf
plot(resultingPlotting(:,1),resultingPlotting(:,2),".")
[pks,potentialRotations] = findpeaks(smooth(resultingPlotting(:,2)),resultingPlotting(:,1));
%potentialRotations = 2*pi-potentialRotations;
display(potentialRotations)

hold on
currentPeakXValue = potentialRotations(1);
y = resultingPlotting(find(resultingPlotting(:,1)==currentPeakXValue)-5:1:find(resultingPlotting(:,1)==currentPeakXValue)+5,2);
x = resultingPlotting(find(resultingPlotting(:,1)==currentPeakXValue)-5:1:find(resultingPlotting(:,1)==currentPeakXValue)+5,1);

f = fit(x,y,"gauss1")
plot(f,x,y)




