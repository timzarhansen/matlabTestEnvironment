clc
clear

figure(1)
ptCloud = pcread('/home/tim-linux/dataFolder/gazeboDataScansPCL/scanNumber_3.pcd');
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)
figure(2)
ptCloud = pcread('/home/tim-linux/dataFolder/gazeboDataScansPCL/scanNumber_4.pcd');
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)