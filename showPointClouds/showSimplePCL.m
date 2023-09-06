clc
clear

whichScan = 0;
ourN = 64;
whichFolder = "/home/tim-external/dataFolder/journalPaperDatasets/highNoiseBigMotionKeller/scanNumber_18";
%  0.988821  0.149104         0  -1.72107
% -0.149104  0.988821        -0  -1.47056
%        -0         0         1         0
%         0         0         0         1

figure(9)
% scan1 = pcread("/home/tim-external/dataFolder/ValentinBunkerData/noNoise305_52/scanNumber_2/0_Threshold.ply");
% scan1 = pcread("/home/tim-external/dataFolder/SimulationEnvironment/rotationNoNoiseConsecutiveScans/scanNumber_0/0_Threshold.ply");
% scan1 = pcread("/home/tim-external/dataFolder/ValentinBunkerData/valentinHighNoise52/scanNumber_18/0_Threshold.ply");
scan1 = pcread(whichFolder + "/scan1_"+whichScan+"_"+ourN+"_pcl.pcd");

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
scan2 = pcread(whichFolder + "/scan2_"+whichScan+"_"+ourN+"_pcl.pcd");
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



figure(12)

voxelDataUsed2 = readmatrix(whichFolder + "/scan1_"+whichScan+"_"+ourN+".csv");
N=size(voxelDataUsed2,1);

voxelData2 =zeros(N,N);
for j =1:N
    for i =1:N
        voxelData2(i,j) = voxelDataUsed2((i-1)*N+j);
    end
end

imagesc((voxelData2))
axis image

figure(13)

voxelDataUsed1 = readmatrix(whichFolder + "/scan2_"+whichScan+"_"+ourN+".csv");
N=size(voxelDataUsed1,1);

voxelData1 =zeros(N,N);
for j =1:N
    for i =1:N
        voxelData1(i,j) = voxelDataUsed1((i-1)*N+j);
    end
end

imagesc((voxelData1))
axis image