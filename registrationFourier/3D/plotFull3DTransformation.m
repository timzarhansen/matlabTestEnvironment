clc
clear
volumeViewer close

whichRotation = 0;


magnitudeFFTW1 = readmatrix("csvFiles/magnitudeFFTW1.csv");
phaseFFTW1 = readmatrix("csvFiles/phaseFFTW1.csv");
voxelDataUsed1 = readmatrix("csvFiles/voxelDataFFTW1.csv");

magnitudeFFTW2 = readmatrix("csvFiles/magnitudeFFTW2.csv");
phaseFFTW2 = readmatrix("csvFiles/phaseFFTW2.csv");
voxelDataUsed2 = readmatrix("csvFiles/voxelDataFFTW2.csv");

magnitudeFFTW2Rotated = readmatrix("csvFiles/magnitudeFFTW2Rotated"+whichRotation+".csv");
phaseFFTW2Rotated = readmatrix("csvFiles/phaseFFTW2Rotated"+whichRotation+".csv");
voxelDataUsed2Rotated = readmatrix("csvFiles/voxelDataFFTW2Rotated"+whichRotation+".csv");



N=nthroot(size(voxelDataUsed1,1),3);
NCorrelation = N*2-1;
magnitude1 =zeros(N,N,N);
phase1 =zeros(N,N,N);
voxelData1 =zeros(N,N,N);
magnitude2 =zeros(N,N,N);
phase2 =zeros(N,N,N);
voxelData2 =zeros(N,N,N);
magnitude2Rotated =zeros(N,N,N);
phase2Rotated =zeros(N,N,N);
voxelData2Rotated =zeros(N,N,N);


for j =1:N
    for i =1:N
        for k =1:N

            index = (i-1)*N*N+(j-1)*N+k;
            magnitude1(i,j,k) = magnitudeFFTW1(index);
            phase1(i,j,k) = phaseFFTW1(index);
            voxelData1(i,j,k) = voxelDataUsed1(index);
            magnitude2(i,j,k) = magnitudeFFTW2(index);
            phase2(i,j,k) = phaseFFTW2(index);
            voxelData2(i,j,k) = voxelDataUsed2(index);
            magnitude2Rotated(i,j,k) = magnitudeFFTW2Rotated(index);
            phase2Rotated(i,j,k) = phaseFFTW2Rotated(index);
            voxelData2Rotated(i,j,k) = voxelDataUsed2Rotated(index);
        end
    end
end


volumeViewer(voxelData1)
volumeViewer(voxelData2)
volumeViewer(voxelData2Rotated)

% volumeViewer(phase1)
% volumeViewer(fftshift(magnitude1))
% volumeViewer(phase2)
% volumeViewer(fftshift(magnitude2))

%%

resampledMagintude1 = readmatrix("csvFiles/resampledMagnitudeSO3_1.csv");
resampledMagintude2 = readmatrix("csvFiles/resampledMagnitudeSO3_2.csv");


resampledMagintude1Matrix =zeros(N,N);
resampledMagintude2Matrix =zeros(N,N);



for j =1:N
    for i =1:N
            resampledMagintude1Matrix(i,j) = resampledMagintude1((i-1)*N+j);
            resampledMagintude2Matrix(i,j) = resampledMagintude2((i-1)*N+j);
    end
end

figure(1)
imagesc(resampledMagintude1Matrix);
axis image
figure(2)
imagesc(resampledMagintude2Matrix);
axis image

figure(3)

sphere(1000);
ch = get(gca,'children');
set(ch,'facecolor','texturemap','cdata',resampledMagintude1Matrix,'edgecolor','none');
axis equal

figure(4)

sphere(1000);
ch = get(gca,'children');
set(ch,'facecolor','texturemap','cdata',resampledMagintude2Matrix,'edgecolor','none');
axis equal



%%


correlationValuesReal = readmatrix("csvFiles/resultingCorrelationReal.csv");
correlationValuesComplex = readmatrix("csvFiles/resultingCorrelationComplex.csv");

% NCorrelation=nthroot(size(correlationValuesReal,1),3);
correlationValuesRealMatrix =zeros(N,N,N);
correlationValuesComplexMatrix =zeros(N,N,N);
correlationValuesMagnitude =zeros(N,N,N);

for j =1:N
    for i =1:N
        for k = 1:N
            correlationValuesRealMatrix(i,j,k) = correlationValuesReal((i-1)*N+j+(k-1)*N*N);
            correlationValuesComplexMatrix(i,j,k) = correlationValuesComplex((i-1)*N+j+(k-1)*N*N);
            correlationValuesMagnitude(i,j,k) = sqrt(correlationValuesRealMatrix(i,j,k)^2 +correlationValuesComplexMatrix(i,j,k)^2);
        end
    end
end


% volumeViewer(correlationValuesRealMatrix)
% volumeViewer(correlationValuesComplexMatrix)
volumeViewer(correlationValuesMagnitude)

%% correlation 3D

translationCorrelation = readmatrix("csvFiles/resultingCorrelationShift"+whichRotation+".csv");
N=nthroot(size(translationCorrelation,1),3);
translationCorrelation3D =zeros(N,N,N);


for j =1:N
    for i =1:N
        for k = 1:N
            index = (i-1)*N*N+(j-1)*N+k;
            translationCorrelation3D(i,j,k) = translationCorrelation(index);
        end
    end
end

volumeViewer(translationCorrelation3D)


[C,I] = max(translationCorrelation3D(:));
C
translationCorrelation3D(I)
[I1,I2,I3] = ind2sub(size(translationCorrelation3D),I)



%%
matrixgt = [
4.18673825e-01	 -3.86012943e-01	  8.22012020e-01	 -2.30746099e+00	
 1.52659542e-01	  9.22199742e-01	  3.55305747e-01	 -7.49586011e-01	
-8.95209483e-01	 -2.32701749e-02	  4.45030372e-01	  1.49160709e+00	
 0.00000000e+00	  0.00000000e+00	  0.00000000e+00	  1.00000000e+00	]

matrix3dmatch = [0.4304742850	-0.3943743280	0.8118871720	-2.3416667600
0.1718689110	0.9188526370	0.3552054450	-0.7486099270
-0.8860885770	-0.0133686460	0.4633231190	1.3367752900
0.0000000000	0.0000000000	0.0000000000	1.0000000000]

anglesR(matrixgt(1:3,1:3),'xyz')

anglesR(matrix3dmatch(1:3,1:3),'xyz')




