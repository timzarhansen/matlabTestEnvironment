clc
clear

whichKeyframe =6;%5
%nameOfFolder = '/home/tim-linux/dataFolder/newStPereDatasetCorrectionOnly/';
nameOfFolder = '/home/tim-linux/dataFolder/gazeboCorrectedPCLs/';
firstScan=['pclKeyFrame',num2str(whichKeyframe),'.pcd'];
secondScan =['pclKeyFrame',num2str(whichKeyframe+1),'.pcd'];



figure(1)
ptCloud = pcread([nameOfFolder firstScan]);
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)
figure(2)
ptCloud = pcread([nameOfFolder secondScan]);
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)