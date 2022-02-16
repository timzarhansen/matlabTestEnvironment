function [voxelData] = getVoxelData(numberOfPoints,shift,rotation,pointCloud,fromTo)
%GETVOXELDATA Summary of this function goes here
%   Detailed explanation goes here
voxelData = zeros(numberOfPoints,numberOfPoints,numberOfPoints);

for j=1:pointCloud.Count
    positionPoint = [pointCloud.Location(j,1)+shift(1),pointCloud.Location(j,2)+shift(2),pointCloud.Location(j,3)+shift(3)]';
    positionPoint = rotation*positionPoint;
    xIndex = cast((positionPoint(1) + fromTo)/(fromTo*2) * numberOfPoints,'int16');
    yIndex = cast((positionPoint(2) + fromTo)/(fromTo*2) * numberOfPoints,'int16');
    zIndex = cast((positionPoint(3) + fromTo)/(fromTo*2) * numberOfPoints,'int16');
    voxelData(yIndex,xIndex,zIndex) = 1;
end

end

