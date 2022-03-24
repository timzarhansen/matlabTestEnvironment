clc
clear

BehaviorDirectory=['datasetGenerated'];
logfilename='_040825_1735_DVL.log';
opts = delimitedTextImportOptions;
opts.Delimiter = {'\t', ' '};

dvlDataRaw = readmatrix([BehaviorDirectory,'/',logfilename],opts);
% logTime (s), year, month, day, hour, minute, second, waterVelX (cm/s), waterVelY (cm/s), waterVelZ (cm/s), waterVelStatus, 
% bottomVelX (cm/s), bottomVelY (cm/s), bottomVelZ (cm/s), bottomVelStatus, rangeBottom1, rangeBottom2, rangeBottom3,
% amplitude1, amplitude2, amplitude3, percentGoodPings, heading, pitch, roll, temperature, pressure, powerLevel, slosv, elosv, distanceTravelX, distanceTravelY

%%
%dvlDataRaw = readtable([BehaviorDirectory,'/',logfilename],'Delimiter',{'\t', ' '})
correctIndexedDVLData = dvlDataRaw(5:end,1:end-1);
correctIndexedDVLData(:,[2,34,35,36,37,38,39]) = [];
DVLData = str2double(regexprep(correctIndexedDVLData, '\t', ''));

cleanedDVLData = DVLData;

cleanedDVLData=cleanedDVLData(logical(cleanedDVLData(:,15)),:);
%cleanedDVLData=cleanedDVLData(logical(cleanedDVLData(:,11)),:);

%logTime (s), bottomVelX (cm/s), bottomVelY (cm/s), bottomVelZ (cm/s),
%rangeBottom1, rangeBottom2, rangeBottom3, heading, pitch, roll
cleanedDVLData(:,[2,3,4,5,6,7,8,9,10,11,15,19,20,21,22,26,27,28,29,30,31,32]) = [];

cleanedDVLData(:,2:4) = cleanedDVLData(:,2:4)/100;
cleanedDVLData(:,8:10) = cleanedDVLData(:,8:10)/180*3.14159;
cleanedDVLDataTable = array2table(cleanedDVLData);
cleanedDVLDataTable.Properties.VariableNames(1:10) = {'logTime (s)', 'bottomVelX (m/s)', 'bottomVelY (m/s)', 'bottomVelZ (m/s)','rangeBottom1', 'rangeBottom2', 'rangeBottom3', 'heading', 'pitch', 'roll'};
writetable(cleanedDVLDataTable,'datasetGenerated/dvlData.csv')










