clc
clear
clf

% 1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform,
    % 6: Our FMS 2D 7: FMS hamming 8: FMS none
    % 9: Feature0 10: Feature1  11: Feature2 12: Feature3 13: Feature4 14: Feature5
    % 15: gmmRegistrationD2D 16: gmmRegistrationP2D 
    
    % 1 whichMethod 2: voxelSize 3: initialGuess 4: errorInDistance 5: errorInRotation 6: calculationTime 7: overlap

numberOfCombinedDatasets = 2;
method_Of_interest = 6;
voxelSize = 256;
initualGuess = 0;
usePoints = 0;
sizeGrid = 50;
listOfOrdner = [
%     "highNoiseBigMotionKeller";
%     "highNoiseBigMotionValentin";
%     "noNoiseSmallMotionValentin";
%     "noNoiseSmallMotionKeller";
%     "onlyRotationNoNoiseValentin";
%     "onlyRotationNoNoiseKeller";
%     "overlapTests";
%     "angleTests";
%     "overlapTests2";
%     "noNoiseBoth";
%     "onlyAngleBoth";
    "highNoiseBoth"

    ];
% liftOfVoxelSize = [256];

liftOfVoxelSize = [64,128,256];

listMethodType = [
    1,1;
    1,2;
    1,3;
    1,4;
    1,5;
    1,6;
    0,6;
    1,7;
    0,7;
    1,8;
    0,8
    0,9;
    0,10;
    0,11;
    0,12;
    0,13;
    0,14;
    0,15;
    0,16;
];

for i = 1:size(listMethodType,1)
    for j = 1:size(listOfOrdner,1)
        for k = 1:size(liftOfVoxelSize,2)

            plotPdfs(listMethodType(i,2),liftOfVoxelSize(k),listMethodType(i,1),listOfOrdner(j),usePoints,sizeGrid,numberOfCombinedDatasets);

        end
    end
end

% ordner = "highNoiseBigMotionKeller";
% ordner = "highNoiseBigMotionValentin";
% ordner = "noNoiseSmallMotionValentin";
% ordner = "noNoiseSmallMotionKeller";
% ordner = "onlyRotationNoNoiseValentin";
% ordner = "onlyRotationNoNoiseKeller";


% plotPdfs(method_Of_interest,voxelSize,initualGuess,ordner)






