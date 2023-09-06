clc
clear

% 1 GICP Error, 2 GICP Time, 3 Super4PCS Error, 4 Super4PCS Time, 5 NDT D2D 2D Error, 6 NDT D2D 2D Time
% 7 NDT P2D Error, 8 NDT P2D Time,9 Our FMS 32 Error, 10 Our FMS 32 Time, 11 Our FMS 64 Error, 12 Our FMS 64,
% 13 Our FMS 128 Error, 14 Our FMS 128 Time, 15 Our FMS 256 Error, 16 Our FMS 256 Time
% 17 Our FMS Fast 32 Error, 18 Our FMS Fast 32 Time, 19 Our FMS Fast 64 Error, 20 Our FMS Fast 64 Time
% 21 Our FMS Fast 128 Error, 22 Our FMS Fast 128 Time, 23 Our FMS Fast 256 Error, 24 Our FMS Fast 256 Time



% resultMatrix = readmatrix("csvFiles/comparisonAllMethodsEvenAngles.csv");
resultMatrix = readmatrix("csvFiles/comparisonAllMethodsOddAngles.csv");

onlyError=resultMatrix(:,1:2:end);
onlyTime=resultMatrix(:,2:2:end);
figure(1);
boxplot(onlyError);
ax = gca;
ax.YAxis.Scale ="log";
xticklabels({'GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D',  'Our FMS 32', 'Our FMS 64', 'Our FMS 128', 'Our FMS 256',  'Our FMS Fast 32',  'Our FMS Fast 64', 'Our FMS Fast 128', 'Our FMS Fast 256'})
title("error(angle diff times norm pos diff)")

figure(2);
boxplot(onlyTime);
ax = gca;
ax.YAxis.Scale ="log";
xticklabels({'GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D',  'Our FMS 32', 'Our FMS 64', 'Our FMS 128', 'Our FMS 256',  'Our FMS Fast 32',  'Our FMS Fast 64', 'Our FMS Fast 128', 'Our FMS Fast 256'})
title("computation Time")
ylabel("computation Time in ms")

