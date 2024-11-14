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



magnitude1=fftshift(magnitude1);
magnitude2=fftshift(magnitude2);

figure(11)
clf

imagesc((magnitude1))
axis image
figure(22)
clf

imagesc((magnitude2))
% phase1=fftshift(phase1);
axis image
% phase2=fftshift(phase2);

figure(1)
clf

title('Voxel: '+string(1))

imagesc((voxelData1))
axis image
% figure 2
figure(2)
clf

title('Voxel: '+string(2))
imagesc((voxelData2))
axis image

imwrite(voxelData1/max(max(voxelData1)),'csvFiles/testImages/image1.jpeg','JPEG');
imwrite(voxelData2/max(max(voxelData2)),'csvFiles/testImages/image2.jpeg','JPEG');
%

dataInformation = readmatrix("csvFiles/dataForReadIn.csv");

numberOfSolutionsOverall = 0;
for i = 1:dataInformation(1)
    numberOfSolutionsOverall=numberOfSolutionsOverall+dataInformation(i+1);
end
cellSize = dataInformation(dataInformation(1)+2);

%
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

subplot( 1, 2, 1 )

imagesc(squeeze(voxelResult1));

title('Voxel 1: ')
axis image

subplot( 1, 2, 2 )
C = imfuse(voxelResult1,voxelResult2,'blend','Scaling','joint');
imagesc(squeeze(C));
title('Voxel 2: ')
axis image
%%

resampledDataForSphere1 = readmatrix("csvFiles/resampledVoxel1.csv");
resampledDataForSphere2 = readmatrix("csvFiles/resampledVoxel2.csv");
size(resampledDataForSphere1,1)
sizeOfSpheres = sqrt(size(resampledDataForSphere1,1));
resampledDataForSphereResult1 =zeros(sizeOfSpheres,sizeOfSpheres);
resampledDataForSphereResult2 =zeros(sizeOfSpheres,sizeOfSpheres);
for j = 1:sizeOfSpheres
    for i =1:sizeOfSpheres
            resampledDataForSphereResult1(j,i) = resampledDataForSphere1((i-1)*sizeOfSpheres+j);
            resampledDataForSphereResult2(j,i) = resampledDataForSphere2((i-1)*sizeOfSpheres+j);
    end
end

% currently not that interesting
if 1
    figure(4)

    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult1,'edgecolor','none');
    axis equal

    figure(5)

    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',resampledDataForSphereResult2,'edgecolor','none');
    axis equal

else
    figure(4)
    imagesc((resampledDataForSphereResult1));
    axis image
    figure(5)
    imagesc((resampledDataForSphereResult2));
    axis image
    
    
end

% read in result
% N=256;
results = readmatrix("csvFiles/resultCorrelation3D.csv");

results = results/max(results);
resultSize = nthroot(length(results),3);

A = reshape(results,resultSize,resultSize,resultSize);
%


correlationNumberMatrix = squeeze(A(:,:,1));
figure(9)
imshow(correlationNumberMatrix)
axis image


%
figure(9)
everyNthElement = 5;
sizeAll = size(A,1);
B = A(1:everyNthElement:end,1:everyNthElement:end,1:everyNthElement:end);
actualSize = ceil(sizeAll/everyNthElement);

[xx, yy, zz] = meshgrid(1:everyNthElement:sizeAll);
vv = cos(xx).*sin(yy).*sin(zz).^2;

scatter3(xx(:),yy(:),zz(:),5,B(:));

colormap(jet);
colorbar;
axis equal
xlabel("z1-axis");
ylabel("z2-axis");
zlabel("y-axis");
%
display("z1: " +string(31/64*360))
display("y: " + string(39/64*360))
display("z2: " + string(45/64*360))

display("z1: " +string(63/64*360))
display("y: " + string(39/64*360))
display("z2: " + string(63/64*360))

display("z1: " +string(31/64*360))
display("y: " + string(39/64*360))
display("z2: " + string(45/64*360))
%
% figure(8)
% clf
lastNumberOfPoints = 0;
figure(6)
clf
correlationOfAngles = readmatrix("csvFiles/resultingCorrelation1D.csv");
xForPlot = 0:size(correlationOfAngles,1)-1;
xForPlot = xForPlot/size(correlationOfAngles,1)*360;
plot(xForPlot,correlationOfAngles)

ylabel("Correlation Value", 'Interpreter', 'latex')
xlabel("angle in rad", 'Interpreter', 'latex')

% nameOfFile = ordnerSave+"correlationValue";
% 
% saveas(gcf,nameOfFile , 'pdf');
% systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
% system(systemCommand);


%%
%calculate 4D quaternion Test
figure(10)
results4D = readmatrix("csvFiles/resultCorrelation4D.csv");

results4D = results4D/max(results4D);
resultSize4D = nthroot(length(results4D),3);

test4D = reshape(results4D,resultSize4D,resultSize4D,resultSize4D);
% test2D = squeeze(test4D(resultSize4D/2,resultSize4D/2,1:everyNthElement:end ,1:everyNthElement:end));


%%
% for i = 1:resultSize4D
%     test2D = squeeze(test4D(i,i,1:everyNthElement:end ,1:everyNthElement:end));
%     figure(10)
%     clf
% 
%     imshow(test2D)
%     display(i)
%     set(gcf,'units','points','position',[50,50,1500,1000])
%     pause(0.3)
% 
% end

%%
% for i = 1:resultSize4D
    everyNthElement = 2;

    test3D = squeeze(test4D(1:everyNthElement:end,1:everyNthElement:end,1:everyNthElement:end));
    sizeAll = size(test4D,1);
%     if max(max(max(test3D)))>0
%             display(i)
%     end
    actualSize = ceil(sizeAll/everyNthElement);
    
    [xx, yy, zz] = meshgrid(1:everyNthElement:sizeAll);
    vv = cos(xx).*sin(yy).*sin(zz).^2;
    figure(10)
    h = scatter3(xx(:),yy(:),zz(:),5,test3D(:));
    
    colormap(jet);
    colorbar;
    alpha = 0.2;
    set(h, 'MarkerEdgeAlpha', alpha, 'MarkerFaceAlpha', alpha)
    axis equal
    xlabel("z1-axis");
    ylabel("z2-axis");
    zlabel("y-axis");
%     display(i)
    pause(0.01)
% end

% figure(11)
% plot(squeeze(test4D(sizeAll/2,sizeAll/2,1:end-1)))


%%
for currentNumberCorrelation = 0:dataInformation(1)-1
    correlationMatrixShift1D = readmatrix("csvFiles/resultingCorrelationShift_"+currentNumberCorrelation+"_.csv");
    resultSize = nthroot(length(correlationMatrixShift1D),2);
    %A = reshape(results,resultSize,resultSize);
    correlationMatrixShift2D =zeros(resultSize);
    for j = 1:resultSize
        for i =1:resultSize
                correlationMatrixShift2D(i,j) = correlationMatrixShift1D((i-1)*resultSize+j);
                correlationMatrixShift2D(i,j) = correlationMatrixShift1D((i-1)*resultSize+j);
        end
    end

    for currentTranslationNumber = 1:dataInformation(currentNumberCorrelation+2)
        figure(8)
        clf
        [Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
        surf(Xplot,Yplot,(correlationMatrixShift2D),'edgecolor', 'none');
        xlabel("x-axis");
        ylabel("y-axis");
        hold on
        display("number of interest:")
        display((currentTranslationNumber+lastNumberOfPoints-1))
        initialGuessReadIn = readmatrix("csvFiles/initialGuess.csv");
        initialGuessReadIn(1,4) = initialGuessReadIn(1,4)/cellSize;
        initialGuessReadIn(2,4) = initialGuessReadIn(2,4)/cellSize;
        xOfInitialGuess = -initialGuessReadIn(1,4)+ceil(resultSize/2);
        yOfInitialGuess = -initialGuessReadIn(2,4)+ceil(resultSize/2);

        zForPlot=correlationMatrixShift2D(round(yOfInitialGuess),round(xOfInitialGuess));
        plot3(xOfInitialGuess,yOfInitialGuess,zForPlot,'r+')


        potentialTransformationReadIn = readmatrix("csvFiles/potentialTransformation"+(currentTranslationNumber-1+lastNumberOfPoints)+".csv");
        transformationTMPSI = potentialTransformationReadIn(1:4,1:4);
        potentialTransformationReadIn(1,4) = potentialTransformationReadIn(1,4)/cellSize;
        potentialTransformationReadIn(2,4) = potentialTransformationReadIn(2,4)/cellSize;
        correlationMatrix = potentialTransformationReadIn(6:7,1:2);
        [V,D] = eig(correlationMatrix);




        transformationTMPPixel = potentialTransformationReadIn(1:4,1:4);

        potentialPeakHeightTMP = potentialTransformationReadIn(5,1);
        xOfInterest = -potentialTransformationReadIn(1,4)+ceil(resultSize/2);
        yOfInterest = -potentialTransformationReadIn(2,4)+ceil(resultSize/2);


        quiver(xOfInterest,yOfInterest,(D(1,1))*V(1,1),(D(1,1))*V(1,2), 0)

        quiver(xOfInterest,yOfInterest,(D(2,2))*V(1,2),(D(2,2))*V(2,2), 0)
        zForPlot=correlationMatrixShift2D(round(yOfInterest),round(xOfInterest));
        plot3(xOfInterest,yOfInterest,zForPlot,'r+')
        r1=2*sqrt(D(1,1));
        r2=2*sqrt(D(2,2));
        teta=-pi:0.01:pi;
        ellipse_x_r=r1*cos(teta);
        ellipse_y_r=r2*sin(teta);
        phi = atan2(V(1,2),V(1,1));
        R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
        r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
        plot3(r_ellipse(:,1)+xOfInterest,r_ellipse(:,2)+yOfInterest,zForPlot*ones(1,numel(teta)))

        set(gca, 'DataAspectRatio', [repmat(min(diff(get(gca, 'XLim')), diff(get(gca, 'YLim'))), [1 2]) diff(get(gca, 'ZLim'))]) 
%         axis equal
%         daspect([max(daspect)*[1 1] 1]) 
        view(0,90)


        figure(7)
        clf

        voxelDataTMP1 = voxelData1;
        voxelDataTMP2 = voxelData2;
        subplot( 1, 2, 1 )

        imagesc(squeeze(imfuse(voxelDataTMP1,voxelDataTMP2,'blend')));
%         imagesc(squeeze(voxelDataTMP2));

        title('Before Match: ')
        axis image

        subplot( 1, 2, 2 )
        angleOfInterest = atan2(transformationTMPPixel(2,1),transformationTMPPixel(1,1));
        voxelDataTMP2 = imtranslate(voxelDataTMP2,[-transformationTMPPixel(1,4), -transformationTMPPixel(2,4)]);
        voxelDataTMP2 = imrotate(voxelDataTMP2,angleOfInterest*180/pi,'bilinear','crop');


        imagesc(squeeze(imfuse(voxelDataTMP1,voxelDataTMP2,'blend')))
%         imagesc(squeeze(voxelDataTMP2));
        numberTMP = potentialPeakHeightTMP/(10^1);
        title('Hight of Peak: '+string(numberTMP))
        axis image
        figure(77)
        imagesc(squeeze(imfuse(voxelDataTMP1,voxelDataTMP2,'blend')))
        axis image
        ordnerSave = "/home/ws/matlab/registrationFourier/resultsJournalFMS2D/pdffigures/";
        nameOfMap = "blendedScansTogether";
        nameOfFile = ordnerSave+nameOfMap;
        
        set(groot,'defaultAxesTickLabelInterpreter','latex'); 
        set(groot,'defaulttextinterpreter','latex');
        set(groot,'defaultLegendInterpreter','latex');
        
        saveas(gcf,nameOfFile , 'pdf');
        addCommandToBatchfile('batchfile.sh','pdfcrop ' + nameOfFile + '.pdf '+nameOfFile+ '.pdf &',true);

        pause(0.01)
        %display((transformationTMPSI))
        display(D)
        display(V)
        display(correlationMatrix)
        display(angleOfInterest/pi*180)
        display("test")

    end
    lastNumberOfPoints = lastNumberOfPoints+currentTranslationNumber;
    display("next")
end

%% calc correct correlation value
dimensionTMP = 511;
a = 31;
b = 31;
tmpCalculation = 0;
if a<ceil(511/2)
    tmpCalculation = dimensionTMP*dimensionTMP*(a);
else
    tmpCalculation = dimensionTMP*dimensionTMP*(dimensionTMP-a+1);
end

if b<ceil(511/2)
    tmpCalculation = tmpCalculation*(b);
else
    tmpCalculation = tmpCalculation*(dimensionTMP-b+1);
end

tmpCalculation

(256*256*511*511)/tmpCalculation

%

%
ourTestArray = correlationMatrixShift2D;
% ourTestArray = medfilt2(ourTestArray,[10 10]);
normalizedArray = ourTestArray/max(max(ourTestArray));
normalizedArray = imgaussfilt(normalizedArray,1);

normalizedArray = imtophat(normalizedArray,strel('disk',5));
normalizedArray=imextendedmax(normalizedArray,10);

s=regionprops(normalizedArray,'Centroid','PixelIdxList');
% ourTestArray = imextendedmax(ourTestArray,max(max(ourTestArray))/10);
% ourTestArray = imadjust(ourTestArray);



p = Fast2DPeakFind(normalizedArray,0.1)
for k = 1:size(p(1:2:end),1)
    display(k)
    zForPlot(k)=normalizedArray(p(2*k),p(2*k-1));

end

figure(9)
clf
% imagesc(ourTestArray)
hold on 
plot3(p(1:2:end),p(2:2:end),zForPlot,'r+')
clear zForPlot
[Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
surf(Xplot,Yplot,(normalizedArray),'edgecolor', 'none');
xlabel("x-axis");
ylabel("y-axis");
set(gca, 'DataAspectRatio', [repmat(min(diff(get(gca, 'XLim')), diff(get(gca, 'YLim'))), [1 2]) diff(get(gca, 'ZLim'))]) 
%%
figure(9),clf,hold on



im=correlationMatrixShift2D;
im = im/max(max(im));



% blur
% im1=imgaussfilt(im,1);
im1=im;
% tophat transform
im2=imtophat(im1,strel('disk',ceil(0.02*resultSize)));

% extended maximums
im3=imextendedmax(im2,0.03);

% Extract each blob
s=regionprops(im3,'Centroid','PixelIdxList');


[Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
surf(Xplot,Yplot,(im),'edgecolor', 'none');
xlabel("x-axis");
ylabel("y-axis");
%
% set(gca, 'DataAspectRatio', [repmat(min(diff(get(gca, 'XLim')), diff(get(gca, 'YLim'))), [1 2]) diff(get(gca, 'ZLim'))]) 

imagesc(im2)
%
for i=1:numel(s)
    x=ceil(s(i).Centroid);
    tmp=im*0;
    tmp(s(i).PixelIdxList)=1;
    tmp2=tmp.*im2;

% The maximum amplitude and location

    [refV,b]=max(tmp2(:));
    [x2,y2]=ind2sub(size(im),b);

% select the region around local max amplitude    
    tmp=bwselect(im2>refV*0.6,y2,x2,4);  

    [xi,yi]=find(tmp);
    zi = zeros(size(xi));
    for j = 1:size(xi,1)
        zi(j) = im(xi(j),yi(j));
    end
    [value,indexMaxZ] = max(zi);
%     mean(im(xi,yi))
    hold on, plot3(yi(indexMaxZ),xi(indexMaxZ),zi(indexMaxZ),'r.')
    hold on, text(y2+10,x2,num2str(i),'Color','white','FontSize',16)    
end


%%

figure(10)
[Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
surf(Xplot,Yplot,(im2),'edgecolor', 'none');
xlabel("x-axis");
ylabel("y-axis");

% imagesc(im2)

% figure(11)
% tmpImage = imhmax(im2,0.05);
% % tmpImage = im3;
% [Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
% surf(Xplot,Yplot,(tmpImage),'edgecolor', 'none');
% xlabel("x-axis");
% ylabel("y-axis");
% imagesc(tmpImage)

figure(12)
imagesc(im3)



%%






