clc
clear

pointCloud = pcread("after_voxel_second.pcd");


%% create voxel grid of pcl %%
fromTo = 30;
numberOfPoints=64;
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

%% create voxel grid of pcl and shift by value%%
voxelData2 = zeros(numberOfPoints,numberOfPoints,numberOfPoints);
shiftfirst = [0,0,0];
rotation = rotationMatrix(0,0,1);
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


%% calculate sampled f(theta,phi)

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

%% calculate fourie coeff of fThetaPhi1
flm1 = fourieCoeff(fThetaPhi1,numberOfPoints,B,theta,phi);

% calculate fourie coeff of fThetaPhi2
flm2 = fourieCoeff(fThetaPhi2,numberOfPoints,B,theta,phi);



%%
NTest = 10;
yawTMP=linspace(0,1.8,NTest);
COutput = zeros(NTest,1);
for j = 1:NTest

    roll = 0;
    pitch = 0;
    yaw = yawTMP(j);

    for l = 1:2*B
        for m1 = 1:(l+1)
            for m2 = 1:(l+1)
                COutput(j) = COutput(j) + flm1(l,m1)*flm2(l,m2)*(-1)^(m1-m2)*exp(-1i*(m1-1)*roll)*wignerdFunction(pitch,l,(m1-1),(m2-1))*exp(-1i*(m2-1)*yaw);
                if isnan(COutput(j))
                    COutput(j)
                end
            end
        end

    end
    
    
    display("done");
    display(j);
    display(COutput(j));

end










