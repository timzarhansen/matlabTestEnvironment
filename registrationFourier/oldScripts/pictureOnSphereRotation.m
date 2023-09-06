clc
clear

I = imread('cameraman.tif');
fThetaPhi1=I;
fThetaPhi2=circshift(I',0)';
thetaRotation = linspace(0,pi,256);
psiRotation = linspace(0,2*pi,256);
pointsOnSphere = zeros(length(fThetaPhi2),length(fThetaPhi2),3);
rotationofSphere = rotationMatrixZYZ(0.0,0.1,0.8);
% rotate sphere
for j =1:length(fThetaPhi2)
    for k =1:length(fThetaPhi2)
        pointsOnSphere(j,k,:) = [sin(thetaRotation(j))*cos(psiRotation(k)) sin(thetaRotation(j))*sin(psiRotation(k)) cos(thetaRotation(j))];
    end
end
pointsOnSphereAfterRotation=pointsOnSphere;
for j =1:length(fThetaPhi2)
    for k =1:length(fThetaPhi2)
        pointsOnSphereAfterRotation(j,k,:) = (rotationofSphere*squeeze(pointsOnSphereAfterRotation(j,k,:)))';
    end
end

newImage = zeros(length(fThetaPhi2),length(fThetaPhi2),2);
for j =1:length(fThetaPhi2)
    for k =1:length(fThetaPhi2)
        
        thetaNew = acos(pointsOnSphereAfterRotation(j,k,3));
        psiNew = atan2(pointsOnSphereAfterRotation(j,k,2),pointsOnSphereAfterRotation(j,k,1));
        psiNew=mod(psiNew+2*pi,2*pi);
        indexTheta = floor(thetaNew/pi*256)+1;
        indexPsi = floor(psiNew/2/pi*256)+1;
        if(indexTheta>256)
            indexTheta = 256;
        end
        if(indexPsi>256)
            indexPsi = 256;
        end

        newImage(indexTheta,indexPsi,1) = newImage(indexTheta,indexPsi,1)+fThetaPhi2(j,k);
        newImage(indexTheta,indexPsi,2) = newImage(indexTheta,indexPsi,2)+1;

    end
end


for j =1:length(fThetaPhi2)
    for k =1:length(fThetaPhi2)
        if(newImage(j,k,2)>0)
            newImage(j,k,1) = newImage(j,k,1)/newImage(j,k,2);
        end
    end
end


fThetaPhi2 = squeeze(newImage(:,:,1));


if 1
    figure(3)
    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',fThetaPhi1,'edgecolor','none');
    axis equal
    
    
    figure(4)
    
    sphere(1000);
    ch = get(gca,'children');
    set(ch,'facecolor','texturemap','cdata',fThetaPhi2,'edgecolor','none');
    axis equal
else
    figure(3)
    imagesc((fThetaPhi1));
    axis image
    
    
    figure(4)
    imagesc((fThetaPhi2));
    axis image

end



fThetaPhi1Interleaved = matrix2InterleavedFormat(fThetaPhi1);
fThetaPhi2Interleaved = matrix2InterleavedFormat(fThetaPhi2);

writematrix(fThetaPhi1Interleaved',"matrixone.csv")
writematrix(fThetaPhi2Interleaved',"matrixtwo.csv")


%%

results = readmatrix("ergWrap.txt");

results = results/max(results);

A = reshape(results,256,256,256);
%%

x = linspace(0,2*pi,256);
y = linspace(0,2*pi,256);
[X,Y] = meshgrid(x,y);
for i = 1:256
    surf(X,Y,squeeze(A(:,:,i)),'edgecolor', 'none');
    xlim([0 7]);
    ylim([0 7]);
    zlim([0.5 1]);
    pause(0.01)
end






