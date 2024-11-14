clc
clear


% consecutiveKeller = readmatrix("oldStuff/consecutiveScansResultsKeller.csv");
% consecutiveTop = readmatrix("oldStuff/consecutiveScansResultTop.csv");
% consecutiveKeller = readmatrix("consecutiveScansKeller2_0.csv");
% consecutiveTop = readmatrix("consecutiveScansTop2_0.csv");
consecutiveKeller = readmatrix("consecutiveScansKeller3_0.csv");
consecutiveTop = readmatrix("consecutiveScansTop3_0.csv");

methods = [1,3,4,5,6,9,10,11,12,13,14,15,16];
% [resultingPercentages1] = readStuff(methods,consecutiveKeller,consecutiveTop,1);
% [resultingPercentages2] = readStuff(methods,consecutiveKeller,consecutiveTop,2);
% [resultingPercentages3] = readStuff(methods,consecutiveKeller,consecutiveTop,3);
% [resultingPercentages4] = readStuff(methods,consecutiveKeller,consecutiveTop,4);
[resultingPercentages5] = readStuff(methods,consecutiveKeller,consecutiveTop,5);




function [resultingPercentages1] = readStuff(methods,consecutiveKeller,consecutiveTop,accuracy)
%ADDCOMMANDTOBATCHFILE Summary of this function goes here
%   Detailed explanation goes here
resultingPercentages1 = zeros(size(methods,2),2);
consecutiveKeller = consecutiveKeller(consecutiveKeller(:,3)==accuracy,:);
for i = 1:size(consecutiveKeller,1)

    indexOfMethod = find(methods==consecutiveKeller(i,1));

    resultingPercentages1(indexOfMethod,1) = resultingPercentages1(indexOfMethod,1)+consecutiveKeller(i,2);
    resultingPercentages1(indexOfMethod,2) = resultingPercentages1(indexOfMethod,2)+1;
end



for i = 1:size(consecutiveTop,1)

    indexOfMethod = find(methods==consecutiveTop(i,1));

    resultingPercentages1(indexOfMethod,1) = resultingPercentages1(indexOfMethod,1)+consecutiveTop(i,2);
    resultingPercentages1(indexOfMethod,2) = resultingPercentages1(indexOfMethod,2)+1;
end



for i = 1:size(resultingPercentages1,1)
    method = methods(i)
    percentage = resultingPercentages1(i,1)/resultingPercentages1(i,2)
end



end


