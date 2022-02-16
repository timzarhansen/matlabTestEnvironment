function [outputVector] = matrix2InterleavedFormat(A)
    %MATRIX2INTERLEAVEDFORMAT Summary of this function goes here
    %   Detailed explanation goes here
    
    %outputVector
    for j = 1:length(A)
        for k = 1:length(A)
            outputVector((length(A)*j-length(A))+k) = A(j,k);
        end
    end
end

