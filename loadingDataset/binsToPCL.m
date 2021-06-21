function [pcl,binsCleaned] = binsToPCL(bins,xPos,yPos,eraseFirstNentries)

maxValueEnd = 30;
threshold = 30;

threshholdIfPointIsThere = 45;

binsCleaned = bins(:,eraseFirstNentries:end);
binsCleaned = smoothdata(binsCleaned,2);



binsCleaned = binsCleaned-linspace(0,maxValueEnd,size(binsCleaned,2)).*ones(size(binsCleaned,1),1);

binsCleaned(binsCleaned<threshold)=0;

%only max pin saved
% [M,index]=max(binsCleaned,[],2);
% for i = 1:size(index,1)
%     if index(i)>1
%         pointsPCL(i,1) = xPos(i,index(i));
%         pointsPCL(i,2) = yPos(i,index(i));
%         pointsPCL(i,3) = 0;
%     end
% end


% pins over threshold saved

[rowOccupied,colOccupied] = find(binsCleaned>threshholdIfPointIsThere);
pointsPCL = zeros(size(rowOccupied,1),3);

for i = 1:size(rowOccupied,1)
    pointsPCL(i,1) = xPos(rowOccupied(i),colOccupied(i));
    pointsPCL(i,2) = yPos(rowOccupied(i),colOccupied(i));
    pointsPCL(i,3) = 0;
end

pcl = pointCloud(pointsPCL);

end

