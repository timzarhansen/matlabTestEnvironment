clc
clear
load('datasetSonar.mat')




%find first scan
for i = 1:size(SonarDataRaw,1)
    if(SonarDataRaw(i,2)==0)
        break
    end
end

startFirstScan=i;

currentStart = startFirstScan;
%%
eraseFirstNentries = 10;%erases the first 50 cm of the bins

j = 1;
while size(SonarDataRaw,1)-currentStart>200
    beginningTimeStamp = SonarDataRaw(currentStart,1);
    bins(1,1:500) = SonarDataRaw(currentStart,3:502);
    angle(1) = SonarDataRaw(currentStart,2);
    i=1;
    while SonarDataRaw(currentStart+i,2)~=0
        bins(i,1:500) = SonarDataRaw(currentStart+i,3:502);
        angle(i)=SonarDataRaw(currentStart+i,2);
        i=i+1;
    end
    bins(i,1:500) = SonarDataRaw(currentStart+i,3:502);
    angle(i)=SonarDataRaw(currentStart+i,2);
    endTimeStamp = SonarDataRaw(currentStart+i,1);
    currentStart = currentStart+i;
    % save current pcl

    positionMatrix = linspace(0.1,50,500).*ones(size(angle,2),1);
    positionMatrix = positionMatrix(:,eraseFirstNentries:end);
    x=positionMatrix.*cos(angle');
    y=positionMatrix.*sin(angle');
    [pcl,binsCleaned]=binsToPCL(bins,x,y,eraseFirstNentries);
    figure(1)
    h = surf(x,y,binsCleaned);
    set(h,'LineStyle','none')
    colormap jet
    axis equal
    view(2)
    figure(2)
    pcwrite(pcl,"datasetGenerated/pclKeyFrame"+j+"TMP.pcd")
    system("pdal translate datasetGenerated/pclKeyFrame"+j+"TMP.pcd datasetGenerated/pclKeyFrame"+j+".pcd --writers.pcd.order=""x=Float:4,y=Float:4,z=Float:4""")
    system("sed -i 's/FIELDS X Y Z/FIELDS x y z/g' datasetGenerated/pclKeyFrame"+j+".pcd")
    system("rm datasetGenerated/pclKeyFrame"+j+"TMP.pcd")
    pcshow(pcl)
    savingKeyframeTimeStamps(j,1:3)=[j,beginningTimeStamp,endTimeStamp];
    j=j+1;
    
    
end

savingKeyframeTimeStampsTable = array2table(savingKeyframeTimeStamps);
savingKeyframeTimeStampsTable.Properties.VariableNames(1:3) = {'keyFrame', 'keyFrameBeginning', 'keyFrameEnd'};
writetable(savingKeyframeTimeStampsTable,'datasetGenerated/keyFrameTimeStamps.csv')


