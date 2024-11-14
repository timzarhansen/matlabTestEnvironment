clc
clear

whichScan = 0;
ourN = 64;
whichFolder = "/home/ws/matlab/showPointClouds/final/";
%  0.988821  0.149104         0  -1.72107
% -0.149104  0.988821        -0  -1.47056
%        -0         0         1         0
%         0         0         0         1

figure(9)
scan1 = pcread(whichFolder +"firstPCL.pcd");

% scan1 = pcread("scan1.ply");
pcshow(scan1);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)


figure(10)
% scan2 = pcread("/home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_2/0_ThresholdShifted.ply");
% scan2 = pcread("/home/tim-external/dataFolder/SimulationEnvironment/rotationNoNoiseConsecutiveScans/scanNumber_0/0_ThresholdShifted.ply");
% scan2 = pcread("/home/tim-external/dataFolder/ValentinBunkerData/testNew/scanNumber_18/0_ThresholdShifted.ply");
scan2 = pcread(whichFolder +"secondPCL.pcd");
% scan2 = pcread("scan2.ply");

pcshow(scan2);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)



figure(11)
scan3 = pcread(whichFolder +"aligned.pcd");
pcshowpair(scan1, scan3)
% pcshow(scan3);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)
