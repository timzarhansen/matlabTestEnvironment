function [flm] = fourieCoeff(fThetaPhi,lMax,B,theta,phi)


lArray = 1:lMax;


b=1;
a=1;
flm=zeros(lMax,lMax+1);
for j=1:2*B
    for k=1:2*B
        for l=lArray
            
            flm(l,1:(l+1))=flm(l,1:(l+1))+a*fThetaPhi(j,k)*YLMofTP(l,theta(j),phi(k));
            
        end

    end 
end

flm = sqrt(2*pi)/(2*b)*flm;
end

