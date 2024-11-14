clc
clear
% nameOfTheDataset="15m4StepsLowSpeed";
% nameOfTheDataset="15m4StepsFastSpeed";
% nameOfTheDataset="7m1StepsSlowSpeed";
% nameOfTheDataset="7m1StepsFastSpeed";
% nameOfTheDataset="7m4StepsFastSpeed";
% nameOfTheDataset="7m4StepsSlowSpeed";
% nameOfTheDataset="15m1StepsSlowSpeed";
% nameOfTheDataset="15m1StepsFastSpeed";

% nameOfTheDataset="micron15m15m";
% nameOfTheDataset="micron7m15m";
% nameOfTheDataset="micron20m20m";

% nameOfTheDataset="tracking15m2step";
nameOfTheDataset="tracking7m2step";


nameOfBatchFile = "batchfile.sh";

nameOfTheDatasetList = ["15m4StepsLowSpeed",
"15m4StepsFastSpeed",
"7m1StepsSlowSpeed",
"7m1StepsFastSpeed",
"7m4StepsFastSpeed",
"7m4StepsSlowSpeed",
"15m1StepsSlowSpeed",
"15m1StepsFastSpeed",
"micron15m15m",
"micron7m15m",
"micron20m20m",
"tracking15m2step",
"tracking7m2step"
];

nameOfTheTitelList = ["3",
"4",
"5",
"6",
"8",
"7",
"1",
"2",
"9",
"10",
"11",
"12",
"13"
];


for k = 1:size(nameOfTheDatasetList,1)

    plotPDFs(nameOfTheDatasetList(k),nameOfBatchFile,nameOfTheTitelList(k));

end




