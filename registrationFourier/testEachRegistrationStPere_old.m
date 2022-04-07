clc
clear
for j=1:229
    whichKeyframe =j;
    %nameOfFolder = '/home/tim-linux/dataFolder/newStPereDatasetCorrectionOnly/';
    nameOfFolder = '/home/tim-linux/dataFolder/gazeboCorrectedPCLs/';
    firstScan=['pclKeyFrame',num2str(whichKeyframe),'.pcd'];
    secondScan =['pclKeyFrame',num2str(whichKeyframe+1),'.pcd'];

    command =['rosrun underwaterslam registrationOfTwoPCLs ',nameOfFolder,firstScan,' ',nameOfFolder,secondScan];
    %system(cmdStr);
    system(command);

    
    %%
    
    
    
    magnitudeFFTW1 = readmatrix("magnitudeFFTW1.csv");
    N=sqrt(size(magnitudeFFTW1,1));
    
    phaseFFTW1 = readmatrix("phaseFFTW1.csv");
    voxelDataUsed1 = readmatrix("voxelDataFFTW1.csv");
    
    magnitudeFFTW2 = readmatrix("magnitudeFFTW2.csv");
    phaseFFTW2 = readmatrix("phaseFFTW2.csv");
    voxelDataUsed2 = readmatrix("voxelDataFFTW2.csv");
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
    
    subplot( 1, 3, 2 )
    
    imagesc(squeeze(magnitude1));
    
    
    %imshow(magnitude2(:,:,size(magnitude2,1)/2));
    title('Magnitude Voxel: '+string(1))
    axis image
    
    subplot( 1, 3, 3 )
    imagesc(phase1);
    %imagesc(squeeze(phase(size(phase,1)/2,:,:)));
    title('Phase Voxel: '+string(1))
    axis image
    
    
    subplot( 1, 3, 1 )
    
    %N = nthroot(length(voxelDataUsed),3);
    
    imagesc((voxelData1))
    axis image
    % figure 2
    figure(2)
    
    subplot( 1, 3, 2 )
    
    imagesc(squeeze(magnitude2));
    
    
    %imshow(magnitude2(:,:,size(magnitude2,1)/2));
    title('Magnitude Voxel: '+string(1))
    axis image
    
    subplot( 1, 3, 3 )
    imagesc(squeeze(phase2));
    %imagesc(squeeze(phase(size(phase,1)/2,:,:)));
    title('Phase Voxel: '+string(1))
    axis image
    
    
    subplot( 1, 3, 1 )
    
    %N = nthroot(length(voxelDataUsed),3);
    
    imagesc((voxelData2))
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
    
    % currently not that interesting
    if 1
    %     figure(3)
    % 
    %     sphere(1000);
    %     ch = get(gca,'children');
    %     set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult2,'edgecolor','none');
    %     axis equal
    % 
    %     figure(4)
    % 
    %     sphere(1000);
    %     ch = get(gca,'children');
    %     set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
    %     axis equal
    
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
    results = readmatrix("resultCorrelation3D.csv");
    
    results = results/max(results);
    resultSize = nthroot(length(results),3);
    %A = reshape(results,resultSize,resultSize);
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
    
    thetaList = linspace(0,2*pi-2*pi/resultSize,resultSize);
    psiList = linspace(0,2*pi-2*pi/resultSize,resultSize);
    listOfCorrelationAndAngle = zeros(resultSize*resultSize,2);
    for j=1:resultSize
        for k=1:resultSize
            currentAngle = mod(thetaList(j)+psiList(k)+4*pi,2*pi);
            listOfCorrelationAndAngle(j*resultSize-resultSize+k,1)=currentAngle;
            listOfCorrelationAndAngle(j*resultSize-resultSize+k,2)=correlationNumberMatrix(k,j);
        end
    end
    %figure(5)
    
    %plot(listOfCorrelationAndAngle(:,1),listOfCorrelationAndAngle(:,2),".")
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
    % figure(6)
    % clf
    % plot(resultingPlotting(:,1),resultingPlotting(:,2),".")
    % [pks,potentialRotations] = findpeaks(smooth(resultingPlotting(:,2)),resultingPlotting(:,1));
    % %potentialRotations = 2*pi-potentialRotations;
    % display(potentialRotations)
    % 
    % hold on
    % currentPeakXValue = potentialRotations(1);
    
    % y = resultingPlotting(find(resultingPlotting(:,1)==currentPeakXValue)-5:1:find(resultingPlotting(:,1)==currentPeakXValue)+5,2);
    % x = resultingPlotting(find(resultingPlotting(:,1)==currentPeakXValue)-5:1:find(resultingPlotting(:,1)==currentPeakXValue)+5,1);
    % 
    % f = fit(x,y,"gauss1")
    % plot(f,x,y)
    
    
    %%
    figure(6)
    correlationOfAngles = readmatrix("resultingCorrelation1D.csv");
    
    plot(correlationOfAngles)
    %%
    
    
    correlationMatrixShift1D = readmatrix("resultingCorrelationShift.csv");
    resultSize = nthroot(length(correlationMatrixShift1D),2);
    %A = reshape(results,resultSize,resultSize);
    correlationMatrixShift2D = reshape(correlationMatrixShift1D,resultSize,resultSize);
    
    
    % [M,I] = max(correlationMatrixShift2D, [], "all", "linear");
    % [dim1, dim2, dim3] = ind2sub(size(correlationMatrixShift2D),I)
    
    %correlationMatrixShift2D  = squeeze(correlationMatrixShift3D(65,:,:));
    figure(8)
    %imagesc(magnitude);
    [Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
    surf(Xplot,Yplot,(correlationMatrixShift2D));
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
    
    
    figure(5)
    clf
    plot(x,y);
    
    y = [y(howManyRemove+1:end-howManyRemove)];
    x = [x(howManyRemove+1:end-howManyRemove)];
    
    
    hold on 
    
    gaussEqn = string(M)+'*exp(-((x-'+string(shiftXY)+')/c/2)^2)';
    f = fit(x',y,gaussEqn,'StartPoint', [20])
    plot(f,x,y);
    
    
    
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
    pointCloudResult1 = pcread("resultingPCL1.pcd");
    pointCloudResult2 = pcread("resultingPCL2.pcd");
    tform = eye(4);
    tform(1:3,1:3)=rotationMatrix(0,0,0);
    tform(1,4)=0;%x value
    tform(2,4)=0;%y value
    tform=affine3d(tform');
    pointCloudResult1 = pctransform(pointCloudResult1,tform);
    
    
    
    pcshowpair(pointCloudResult1, pointCloudResult2)
    
    
    
    %%
    % figure(4)
    % for i = 2:10
    %     %i = 40;
    %     pcl = pcread("/home/tim-linux/dataFolder/StPereDataset/pclKeyFrame"+i+".pcd");
    %     %pcl = pcread("/home/tim-linux/dataFolder/newStPereDatasetCorrectionOnly/pclKeyFrame"+i+".pcd");
    %     pcshow(pcl)
    %     pause(0.3)
    % 
    % end
end