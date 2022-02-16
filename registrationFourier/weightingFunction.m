function [w] = weightingFunction(B)
    w=zeros(B*2,1);
    
    for j = 1:2*B
        resultSum=0;
        for k=0:B-1
            resultSum = resultSum +1/(2*k+1)*sin((2*j+1)*(2*k+1)*pi/4/B);
        end
        w(j)=2/B*sin(pi*(2*j+1)/4/B)*resultSum;
    end

end













