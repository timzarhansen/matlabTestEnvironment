function [fThetaPhi] = sampledFThetaPhi(magnitude,theta,phi,B,rNumbers)


    fThetaPhi = zeros(size(theta,2),size(phi,2));
    for r=rNumbers
        fThetaPhiTMP = zeros(size(theta,2),size(phi,2));
        for j=1:size(theta,2)
            for k=1:size(phi,2)
            
                u=cast(r*sin(theta(j))*cos(phi(k))+B,'int16');
                v=cast(r*sin(theta(j))*sin(phi(k))+B,'int16');
                w=cast(r*cos(theta(j))+B,'int16');

                fThetaPhiTMP(j,k)=fThetaPhiTMP(j,k)+magnitude(u,v,w);
            end
        end
        fThetaPhi=fThetaPhi+adapthisteq(fThetaPhiTMP,'clipLimit',0.03,'Distribution','rayleigh');
    end

end

