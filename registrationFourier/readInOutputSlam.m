clc
clear
data = readJsonGraphPCL("outputSlam.json");


%% plot two pcl
positionCloudOne = 18;
positionCloudZwo = positionCloudOne+1;

pointcloud1 = squeeze(data(positionCloudOne,:,:));
pointcloud1(pointcloud1(:, 1)== 0 & pointcloud1(:, 2)== 0 & pointcloud1(:, 3)== 0, :)= [];


pointcloud2 = squeeze(data(positionCloudZwo,:,:));
pointcloud2(pointcloud2(:, 1)== 0 & pointcloud2(:, 2)== 0 & pointcloud2(:, 3)== 0, :)= [];


figure(1)
plot(pointcloud1(:,1),pointcloud1(:,2),".")

figure(2)
plot(pointcloud2(:,1),pointcloud2(:,2),".")














