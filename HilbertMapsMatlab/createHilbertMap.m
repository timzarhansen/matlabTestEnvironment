clc
clear

pointCloud = pcread("after_voxel_second.pcd");


%% create datapoints for training %%

%% create fourie features, which map data points to the hilbert space %%

%% calculate the current gradient of function R(w) = lambda1 * sum(w^T * w) + lambda2 * sum(|w1|+|w2|+...+|wn|)

%% train vector w to correctly predict the correct occupacy %%

