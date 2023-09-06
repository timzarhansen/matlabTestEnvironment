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
rotation = rotationMatrix(0.0,0.0,0.1);
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

rNumbers=10:numberOfPoints/2-1;%2:31;%was 4:62
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

% for k=1:numberOfPoints
%     fThetaPhi2(:,k)=fThetaPhi2(:,k).*hamming(numberOfPoints);
%     fThetaPhi1(:,k)=fThetaPhi1(:,k).*hamming(numberOfPoints);
% end


% maximumSet2 = max(fThetaPhi2, [], "all", "linear");
% fThetaPhi1=fThetaPhi1/maximumSet2;
% fThetaPhi2=fThetaPhi2/maximumSet2;


% fThetaPhi1 = adapthisteq(fThetaPhi1,'clipLimit',0.03,'Distribution','rayleigh');
% fThetaPhi2 = adapthisteq(fThetaPhi2,'clipLimit',0.03,'Distribution','rayleigh');

% boxKernel = ones(2,2); % Or whatever size window you want.
% fThetaPhi2 = conv2(fThetaPhi2, boxKernel, 'same');


% figure(3)
% sphere(1000);
% ch = get(gca,'children');
% set(ch,'facecolor','texturemap','cdata',fThetaPhi1,'edgecolor','none');
% axis equal
% 
% 
% figure(4)
% 
% sphere(1000);
% ch = get(gca,'children');
% set(ch,'facecolor','texturemap','cdata',fThetaPhi2,'edgecolor','none');
% axis equal

figure(3)
imagesc((fThetaPhi1));
axis image


figure(4)
imagesc((fThetaPhi2));
axis image


fThetaPhi1Interleaved = matrix2InterleavedFormat(fThetaPhi1);
fThetaPhi2Interleaved = matrix2InterleavedFormat(fThetaPhi2);

writematrix(fThetaPhi1Interleaved',"matrixone.csv")
writematrix(fThetaPhi2Interleaved',"matrixtwo.csv")
%%
% calculate fourie coeff of fThetaPhi1
[flmP1,flmM1] = fourieCoeff(fThetaPhi1,numberOfPoints,B,theta,phi);

% calculate fourie coeff of fThetaPhi2
[flmP2,flmM2] = fourieCoeff(fThetaPhi2,numberOfPoints,B,theta,phi);



%%
yawNumber = 51;
yawArray=linspace(-0.5,0.5,yawNumber);
rollNumber = 1;
rollArray=linspace(-0.0,0.0,rollNumber);
pitchNumber = 1;
pitchArray=linspace(-0.0,0.0,pitchNumber);
COutput = zeros(yawNumber,rollNumber,pitchNumber);
for j = 1:yawNumber
    for i = 1:rollNumber
        for k = 1:pitchNumber
            roll = rollArray(i);
            pitch = pitchArray(k);
            yaw = yawArray(j);
        
            for l = 1:2*B
                for m1 = 0:l
                    for m2 = 0:l
                        if m1>0
                            plusAdd1 = flmM1(l,m1);
                            minusAdd1 = flmP1(l,m1+1);
                        else
                            plusAdd1 = flmP1(l,m1+1);
                            minusAdd1 = 0;
                        end
        
                        if m2>0
                            plusAdd2 = conj(flmM2(l,m2));
                            minusAdd2 = conj(flmP2(l,m2+1));
                        else
                            plusAdd2 = conj(flmP2(l,m2+1));%conj
                            minusAdd2 = 0;
                        end
                        plusWignerdDFunction = exp(-1i*m1*roll)*wignerdFunction(pitch,l,m1,m2)*exp(-1i*m2*yaw);
                        COutput(j,i,k) = COutput(j,i,k) + plusAdd1*plusAdd2*(-1)^(m1-m2)*plusWignerdDFunction;
    
                        minusWignerdDFunction = exp(1i*m1*roll)*wignerdFunction(pitch,l,-m1,-m2)*exp(1i*m2*yaw);
                        COutput(j,i,k) = COutput(j,i,k) + minusAdd1*minusAdd2*(-1)^(-m1+m2)*minusWignerdDFunction;
        
                    end
                end
            end
        end
    end
    
    display("done");
    display(j);
    display(COutput(j,1,1));

end

%                 if isnan(COutput(j))
%                     COutput(j)
%                 end
%%
% clc
% clear
% load('resultRegistration.mat')
% COutputForPrint = abs(real(COutput(:)));
% COutputForPrint = 1-COutputForPrint/max(COutputForPrint);
% 
% 
% 
% yawNumber = 11;
% yawArray=linspace(-0.5,0.5,yawNumber);
% rollNumber = 11;
% rollArray=linspace(-0.5,0.5,rollNumber);
% pitchNumber = 11;
% pitchArray=linspace(-0.5,0.5,pitchNumber);
% [mg3dX, mg3dY, mg3dZ] = meshgrid(yawArray,rollArray,pitchArray);
% scatter3(mg3dX(:),mg3dY(:),mg3dZ(:),20,[COutputForPrint,COutputForPrint,COutputForPrint],'filled')
% 
% 
% axis equal
% xlim([-1 1])
% ylim([-1 1])
% zlim([-1 1])
%%
figure(5)
plot(yawArray,(real(COutput(:,1,1))))

%%
clc
clear
data = fopen('test.txt');
A = textscan(data,'%s','Delimiter','\n');
B = A{1,1};

for i=1:length(B)
    C(i)=str2double(B{i});
end


