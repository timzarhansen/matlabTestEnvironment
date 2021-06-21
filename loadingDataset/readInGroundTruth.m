clc
clear

BehaviorDirectory=['/home/tim/DataForTests/StPereDataset'];
logfilename='_040825_1735_DGPS.log';
opts = delimitedTextImportOptions;
opts.Delimiter = {'\t', ' '};

GTdataRAW = readmatrix([BehaviorDirectory,'/',logfilename],opts);
% time, positionX, positionY, positionZ, velocityX, velocityY, velocityZ, accelerationX, accelerationY, accelerationZ, 
%%

%dvlDataRaw = readtable([BehaviorDirectory,'/',logfilename],'Delimiter',{'\t', ' '})
correctIndexedGTData = GTdataRAW(5:end,1:end-8);


%correctIndexedIMUData(:,[2,6,10]) = [];



GTData = str2double(regexprep(correctIndexedGTData, '\t', ''));
%%

cleanedGTDataTable = array2table(GTData);
cleanedGTDataTable.Properties.VariableNames(1:10) = {'time', 'latitude', 'longitude','status','altitude','geoide altitude','true course (deg)', 'magnetic course (deg)', 'vel (knots)', 'vel (km/h)'};
writetable(cleanedGTDataTable,'datasetGenerated/GTData.csv')










