function [fThetaPhi] = sampledFThetaPhi(magnitude,theta,phi,B,rNumbers)


    fThetaPhi = zeros(size(theta,2),size(phi,2));

    for j=1:size(theta,2)
        for k=1:size(phi,2)
            for r=rNumbers
                u=cast(r*sin(theta(j))*cos(phi(k))+B,'int16');
                v=cast(r*sin(theta(j))*sin(phi(k))+B,'int16');
                w=cast(r*cos(theta(j))+B,'int16');
                fThetaPhi(j,k)=fThetaPhi(j,k)+magnitude(u,v,w);
            end
        end
    end

end

