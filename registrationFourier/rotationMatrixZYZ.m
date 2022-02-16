function [rotation] = rotationMatrixZYZ(r1,r2,r3)
%ROTATIONMATRIX Summary of this function goes here
%   Detailed explanation goes here
rotation  = [cos(r1)*cos(r2)*cos(r3)-sin(r1)*sin(r3) -cos(r3)*sin(r1)-cos(r1)*cos(r2)*sin(r3) cos(r1)*sin(r2);
    cos(r1)*sin(r3) + cos(r2)*cos(r3)*sin(r1) cos(r1)*cos(r3) - cos(r2)*sin(r1)*sin(r3) sin(r1)*sin(r2);
    -cos(r3)*sin(r2) sin(r2)*sin(r3) cos(r2)];
end

