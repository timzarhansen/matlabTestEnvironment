function [rotation] = rotationMatrix(roll,pitch,yaw)
%ROTATIONMATRIX Summary of this function goes here
%   Detailed explanation goes here
rotation  = [cos(yaw)*cos(pitch),cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(pitch),cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
    sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
    -sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll)];
end

