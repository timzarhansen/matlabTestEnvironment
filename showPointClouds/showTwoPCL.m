clc
clear

figure(1)
ptCloud = pcread('firstPCL.pcd');
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)
figure(2)
ptCloud = pcread('secondPCL.pcd');
pcshow(ptCloud);
xlabel('X');
ylabel('Y');
zlabel('Z');
view(0,90)