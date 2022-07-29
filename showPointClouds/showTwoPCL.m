clc
clear

whichKeyframe =6;%5
%nameOfFolder = '/home/tim-external/dataFolder/newStPereDatasetCorrectionOnly/';
nameOfFolder = '/home/tim-external/dataFolder/gazeboCorrectedPCLs/';
firstScan=['pclKeyFrame',num2str(whichKeyframe),'.pcd'];
secondScan =['pclKeyFrame',num2str(whichKeyframe+1),'.pcd'];



figure(9)
%ptCloud = pcread([nameOfFolder firstScan]);
scan1 = pcread("scan1.ply");
pcshow(scan1);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)
figure(10)
% ptCloud = pcread([nameOfFolder secondScan]);
scan2 = pcread("scan2.ply");
pcshow(scan2);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)

figure(11)
% ptCloud = pcread([nameOfFolder secondScan]);

transformMatrix = [    0.983263     0.182193  1.01876e-17      9.45563
   -0.182193     0.983263   4.2202e-19     -4.57677
 -1.0094e-17  1.44116e-18            1 -3.58503e-16
           0            0            0            1];
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
