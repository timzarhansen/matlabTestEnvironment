clc
clear

pointCloud = pcread("after_voxel_second.pcd");


% create voxel grid of pcl %%
fromTo = 30;
numberOfPoints=32;
voxelData1 = zeros(numberOfPoints,numberOfPoints,numberOfPoints);

for j=1:pointCloud.Count
    xPos=pointCloud.Location(j,1);
    yPos=pointCloud.Location(j,2);
    xIndex = cast((xPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    yIndex = cast((yPos + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    zIndex = cast((0 + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    voxelData1(xIndex,yIndex,zIndex) = 1;
end
% 3D:
[spectrum1,magnitude1,phase1] = plotffts(voxelData1,1);
%magnitude1 = fftshift(magnitude1);
% create voxel grid of pcl and shift by value%%
voxelData2 = zeros(numberOfPoints,numberOfPoints,numberOfPoints);
shiftfirst = [0,0,0];
rotation = rotationMatrix(0.0,0.0,0.8);
for j=1:pointCloud.Count
    positionPoint = [pointCloud.Location(j,1)+shiftfirst(1),pointCloud.Location(j,2)+shiftfirst(2),0]';
    positionPoint = rotation*positionPoint;
    xIndex = cast((positionPoint(1) + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    yIndex = cast((positionPoint(2) + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    zIndex = cast((positionPoint(3) + fromTo)/(fromTo*2) *numberOfPoints,'int16');
    voxelData2(xIndex,yIndex,zIndex) = 1;
end

% 3D:
voxelData2=voxelData2 + 0.0*randn(size(voxelData2));
[spectrum2,magnitude2,phase2] =plotffts(voxelData2,2);
%magnitude2 = fftshift(magnitude2);

% calculate sampled f(theta,phi)

rNumbers=4:numberOfPoints/2-1;%2:31;%was 4:62
B=numberOfPoints/2;

thetaIndex = 1:2*B;
phiIndex = 1:2*B;
phi = pi*phiIndex/B;
theta = pi*(2*thetaIndex+1)/(4*B);


maximumSet2 = max(magnitude1, [], "all", "linear");
magnitude1=magnitude1/maximumSet2;
magnitude2=magnitude2/maximumSet2;


% 2D:
fThetaPhi1 = sampledFThetaPhi(magnitude1,theta,phi,B,rNumbers);
fThetaPhi2 = sampledFThetaPhi(magnitude2,theta,phi,B,rNumbers);

for j=1:numberOfPoints
    fThetaPhi2(:,j)=fThetaPhi2(:,j).*hamming(numberOfPoints);
    fThetaPhi1(:,j)=fThetaPhi1(:,j).*hamming(numberOfPoints);
end


% maximumSet2 = max(fThetaPhi2, [], "all", "linear");
% fThetaPhi1=fThetaPhi1/maximumSet2;
% fThetaPhi2=fThetaPhi2/maximumSet2;


% fThetaPhi1 = adapthisteq(fThetaPhi1,'clipLimit',0.03,'Distribution','rayleigh');
% fThetaPhi2 = adapthisteq(fThetaPhi2,'clipLimit',0.03,'Distribution','rayleigh');

% boxKernel = ones(2,2); % Or whatever size window you want.
% fThetaPhi2 = conv2(fThetaPhi2, boxKernel, 'same');


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

% figure(3)
% imagesc((fThetaPhi1));
% axis image
% 
% 
% figure(4)
% imagesc((fThetaPhi2));
% axis image
%%
% calculate fourie coeff of fThetaPhi1
[flmP1,flmM1] = fourieCoeff(fThetaPhi1,numberOfPoints,B,theta,phi);

% calculate fourie coeff of fThetaPhi2
[flmP2,flmM2] = fourieCoeff(fThetaPhi2,numberOfPoints,B,theta,phi);



%%
NTest = 10;
yawTMP=linspace(0,1.0,NTest);
COutput = zeros(NTest,1);
for j = 1:NTest

    roll = 0;
    pitch = 0;
    yaw = yawTMP(j);

    for l = 1:2*B
        for m1 = 0:l
            for m2 = 0:l
                if m1>0
                    plusAdd1 = flmP1(l,m1+1);
                    minusAdd1 = flmM1(l,m1);
                else
                    plusAdd1 = flmP1(l,m1+1);
                    minusAdd1 = 0;
                end

                if m2>0
                    plusAdd2 = flmP2(l,m2+1);
                    minusAdd2 = flmM2(l,m2);
                else
                    plusAdd2 = flmP2(l,m2+1);
                    minusAdd2 = 0;
                end
                    plusWignerdDFunction = exp(-1i*m1*roll)*wignerdFunction(pitch,l,m1,m2)*exp(-1i*m2*yaw);
                    COutput(j) = COutput(j) + plusAdd1*plusAdd2*(-1)^(m1-m2)*plusWignerdDFunction;

                    minusWignerdDFunction = exp(1i*m1*roll)*wignerdFunction(pitch,l,-m1,-m2)*exp(1i*m2*yaw);
                    COutput(j) = COutput(j) + minusAdd1*minusAdd2*(-1)^(-m1+m2)*minusWignerdDFunction;

            end
        end
    end


    
    
    display("done");
    display(j);
    display(COutput(j));

end

%                 if isnan(COutput(j))
%                     COutput(j)
%                 end
% figure(5)
% plot(yawTMP,abs(real(COutput)))

