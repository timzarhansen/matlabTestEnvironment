clc
clear
%homeFolder = "/home/tim-external/dataFolder/StPereDataset/csv_pcd_Data/";
%homeFolder = "/home/tim-external/dataFolder/ValentinBunkerData/4_7_Bunker_range_15_1/";
%homeFolder = "/home/tim-external/dataFolder/ValentinBunkerData/4_7_Bunker_range_30_5/";
homeFolder = "/home/tim-external/dataFolder/ValentinBunkerData/directoryTest/scanNumber_0/";








numberOfScan = 9;
figure(1)
AB = readmatrix(homeFolder + numberOfScan +"intensity64.csv");
imagesc(AB)
figure(2)
AB = readmatrix(homeFolder + numberOfScan +"intensityShifted64.csv");
imagesc(AB)
figure(3)
AB = readmatrix(homeFolder + numberOfScan +"intensity128.csv");
imagesc(AB)
figure(4)
AB = readmatrix(homeFolder + numberOfScan +"intensity256.csv");
imagesc(AB)

figure(5)
withThreashold = pcread(homeFolder + numberOfScan +"_Threashold.ply");
pcshow(withThreashold);



figure(6)

oneValue = pcread(homeFolder + numberOfScan +"_OneValue.ply");
pcshow(oneValue);
