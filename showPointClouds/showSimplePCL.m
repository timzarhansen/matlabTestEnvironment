clc
clear





figure(9)
% scan1 = pcread("/home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_2/0_Threshold.ply");
% scan1 = pcread("/home/tim-external/dataFolder/SimulationEnvironment/rotationNoNoiseConsecutiveScans/scanNumber_0/0_Threshold.ply");
% scan1 = pcread("/home/tim-external/dataFolder/ValentinBunkerData/valentinHighNoise52/scanNumber_18/0_Threshold.ply");
scan1 = pcread("/home/tim-external/dataFolder/2022FinalPaperData/stPereNoNoise52NEW/scanNumber_1/0_Threshold.pcd");

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
scan2 = pcread("/home/tim-external/dataFolder/2022FinalPaperData/stPereNoNoise52NEW/scanNumber_1/0_ThresholdShifted.pcd");
% scan2 = pcread("scan2.ply");

pcshow(scan2);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)



figure(11)
pcshowpair(scan1, scan2)
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)
