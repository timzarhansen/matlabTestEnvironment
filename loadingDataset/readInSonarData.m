clc
clear

BehaviorDirectory=['/home/tim/DataForTests/StPereDataset'];
logfilename='_040825_1735_IS.log';

delimiterIn = ' ';


%SonarDataRaw = importdata([BehaviorDirectory,'/',logfilename],delimiterIn); %Import from the log file


opts = delimitedTextImportOptions;
opts.Delimiter = ' ';
%opts.VariableTypes = 'double';
SonarDataRaw = readmatrix([BehaviorDirectory,'/',logfilename],opts);%% logTime (s), sensorTime (ms), transducerAngle (rad), bins 

SonarDataRaw = str2double(regexprep(SonarDataRaw(8:end,1:end-1), '\t', ''));
%% plot first image of dataset

for i = 1:size(SonarDataRaw,1)
    if(SonarDataRaw(i,2)==0)
        break
    end
end
startFirstScan=i;
bins(1,1:500) = SonarDataRaw(startFirstScan,3:502);
angle(1) = SonarDataRaw(startFirstScan,2);
i=1;
while SonarDataRaw(startFirstScan+i,2)~=0
    bins(i,1:500) = SonarDataRaw(startFirstScan+i,3:502);
    angle(i)=SonarDataRaw(startFirstScan+i,2);
    i=i+1;
end
bins(i,1:500) = SonarDataRaw(startFirstScan+i,3:502);
angle(i)=SonarDataRaw(startFirstScan+i,2);
    



%%
imagesc(bins)


%%
%claculate x y coordinates
eraseFirstNentries = 5;%erases the first 50 cm of the bins
positionMatrix = linspace(0.1,50,500).*ones(size(angle,2),1);
positionMatrix = positionMatrix(:,eraseFirstNentries:end);
x=positionMatrix.*sin(angle');
y=positionMatrix.*cos(angle');

[pcl,binsCleaned]=binsToPCL(bins,x,y,eraseFirstNentries);

figure(1)
h = surf(x,y,binsCleaned);
set(h,'LineStyle','none')
colormap jet
axis equal
view(2)
figure(2)
%pcwrite(ptCloud,"testfilename.pcd")
pcshow(pcl)








