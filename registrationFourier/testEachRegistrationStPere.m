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
    transformation1 = readmatrix([nameOfFolder , 'position',num2str(whichKeyframe),'.csv']); 
    transformation2 = readmatrix([nameOfFolder , 'position',num2str(whichKeyframe+1),'.csv']);
    resultingTranslationDiff = transformation2(1:3)-transformation1(1:3);
    resultingRotationDiff = transformation2(4:6)-transformation1(4:6);
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
    clf
    imagesc(squeeze(magnitude1));
    
    title('Magnitude first PCL Voxel:')
    axis image
    
    figure(2)
    clf
    imagesc((voxelData1))
    axis image
    title('First PointCloud as Voxel:')
    % figure 2
    
    
    figure(3)
    
    imagesc(squeeze(magnitude2));
    
    
    %imshow(magnitude2(:,:,size(magnitude2,1)/2));
    title('Magnitude second PCL Voxel:')
    axis image
    
    
    
    figure(4)
    imagesc((voxelData2))
    axis image
    title('Second PointCloud as Voxel:')
    
    
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
        figure(5)
        clf
    
        sphere(1000);
        ch = get(gca,'children');
        set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult2,'edgecolor','none');
        axis equal
        title('Second Resampled:')
    
        figure(6)
        clf
        sphere(1000);
        ch = get(gca,'children');
        set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
        axis equal
        title('first Resampled:')
    
    else
        figure(5)
        clf
        imagesc((resampledDataForSphereResult2));
        axis image
            title('second Resampled:')
        figure(6)
        clf
        imagesc((resampledDataForSphereResult1));
        axis image
            title('first Resampled:')
        
    end
    
    %%
    figure(7)
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
    
    
    
    figure(9)
    pointCloudResult1 = pcread("resultingPCL1.pcd");
    pointCloudResult2 = pcread("resultingPCL2.pcd");
    tform = eye(4);
    tform(1:3,1:3)=rotationMatrix(0,0,0);
    tform(1,4)=0;%x value
    tform(2,4)=0;%y value
    tform=affine3d(tform');
    pointCloudResult1 = pctransform(pointCloudResult1,tform);
    
    
    
    pcshowpair(pointCloudResult1, pointCloudResult2)






%% calculate difference of Rotation 

    resultingTransformationOfScan = readmatrix("resultingTransformation.csv");





end