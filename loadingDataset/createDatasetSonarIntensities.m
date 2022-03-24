clc
clear
load('datasetSonar.mat')

%%

writematrix(SonarDataRaw,"datasetGenerated/intensitiesData.csv")

%%
for i = 1:3
    %i = 40;
    pcl = pcread("pclKeyFrame"+i+".pcd")
    pcshow(pcl)
    pause(0.3)

end