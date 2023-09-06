function [y] = angleDiff(A,B)
%ANGLEDIFF Summary of this function goes here
%   Detailed explanation goes here

    y=rem(B-A,2*pi); % returns the remainder when dividing by 360
    if y<-pi
        y=y+2*pi;
    elseif y>pi
        y=y-2*pi;
    end

end

