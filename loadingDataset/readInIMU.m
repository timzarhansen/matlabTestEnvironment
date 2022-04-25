clc
clear

BehaviorDirectory=['/home/tim-external/DataForTests/StPereDataset'];
logfilename='_040825_1735_MTi.log';
opts = delimitedTextImportOptions;
opts.Delimiter = {'\t', ' '};

IMUDataRaw = readmatrix([BehaviorDirectory,'/',logfilename],opts);
% time, positionX, positionY, positionZ, velocityX, velocityY, velocityZ, accelerationX, accelerationY, accelerationZ, 
%%

%dvlDataRaw = readtable([BehaviorDirectory,'/',logfilename],'Delimiter',{'\t', ' '})
correctIndexedIMUData = IMUDataRaw(6:end,1:end-2);


correctIndexedIMUData(:,[2,6,10]) = [];


IMUData = str2double(regexprep(correctIndexedIMUData, '\t', ''));
%%

IMUData(:,2:4) = IMUData(:,2:4)/180*3.14159;

cleanedIMUDataTable = array2table(IMUData);
cleanedIMUDataTable.Properties.VariableNames(1:10) = {'time', 'roll (deg)', 'pitch (deg)', 'yaw (deg)', 'rollVel (rad/s)', 'pitchVel (rad/s)', 'yawVel (rad/s)', 'accelX (m/s2)', 'accelY (m/s2)', 'accelZ (m/s2)'};
writetable(cleanedIMUDataTable,'datasetGenerated/IMUData.csv')










