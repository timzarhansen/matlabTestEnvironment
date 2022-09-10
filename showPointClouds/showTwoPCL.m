clc
clear

whichKeyframe =6;%5
%nameOfFolder = '/home/tim-external/dataFolder/newStPereDatasetCorrectionOnly/';
%nameOfFolder = '/home/tim-external/dataFolder/gazeboCorrectedPCLs/';
nameOfFolder = '/home/tim-external/dataFolder/ValentinBunkerData/veryVeryHighNoise305_1510/scanNumber_0/';
%nameOfFolder = '/home/tim-external/dataFolder/StPereDataset/veryHighNoise/scanNumber_10/';
%nameOfFolder = '/home/tim-external/dataFolder/StPereDataset/onlyAngle5/scanNumber_0/';
%nameOfFolder = '/home/tim-external/dataFolder/StPereDataset/onlyAngle25/scanNumber_0/';

%nameOfFolder = '/home/tim-external/dataFolder/ValentinBunkerData/directoryTest/scanNumber_0/'
% firstScan = ['pclKeyFrame',num2str(whichKeyframe),'.pcd'];
% secondScan = ['pclKeyFrame',num2str(whichKeyframe+1),'.pcd'];
%firstScan=['1_Threshold.ply'];
%secondScan =['1_ThresholdShifted.ply'];
numberOfCurrentTHings = 4;

% firstScan=[string(numberOfCurrentTHings)+'_OneValue.ply'];
% secondScan =[string(numberOfCurrentTHings) + '_OneValueShifted.ply'];

firstScan=[string(numberOfCurrentTHings)+'_Threshold.ply'];
secondScan =[string(numberOfCurrentTHings) + '_ThresholdShifted.ply'];

thirdScan = [string(numberOfCurrentTHings) + 'intensityShifted256.csv'];
fourthScan = [string(numberOfCurrentTHings) + 'intensity256.csv'];

figure(9)
scan1 = pcread([nameOfFolder + firstScan]);
%scan1 = pcread("scan1.ply");
pcshow(scan1);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)
figure(10)
scan2 = pcread([nameOfFolder + secondScan]);
%scan2 = pcread("scan2.ply");
pcshow(scan2);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)

figure(11)
% ptCloud = pcread([nameOfFolder secondScan]);

transformMatrix = [1  0        0 0
0 1       0 0
       0        0        1        0
       0        0        0        1];
transformMatrix = (transformMatrix)

rot180 = [1 0 0 0; ...
      0 -1 0 0; ...
               0          0  -1 0;...
               0 0 0 1];

transformMatrix = rot180*transformMatrix


theta = atan2(transformMatrix(1,2),transformMatrix(1,1));

rot = [cos(theta) sin(theta) 0; ...
      -sin(theta) cos(theta) 0; ...
               0          0  1]





tform = rigid3d(rot,[transformMatrix(1,4) , transformMatrix(2,4) , 0]);
scan2 = pctransform(scan2,tform);
%scan1 = pctransform(scan1,tform);


final = pcread("4pcs_fast.ply");
pcshowpair(scan1, scan2)
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)

figure(12)
map = readmatrix([nameOfFolder + thirdScan]);
imagesc(map);

figure(13)
map = readmatrix([nameOfFolder + fourthScan]);
imagesc(map);


