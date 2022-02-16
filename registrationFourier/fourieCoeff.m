function [flmP,flmM] = fourieCoeff(fThetaPhi,lMax,B,theta,phi)
    b=B;
    weights = ones(B*2,1);%weightingFunction(B);
    flmP=zeros(lMax,lMax+1);
    flmM=zeros(lMax,lMax);
    for l=1:lMax
        [YLP ,YLM] = YLMofTP(l,theta,phi);
        for m=0:l
            for j=1:2*B
                for k=1:2*B
                    if m>0
                        flmP(l,m+1) = flmP(l,m+1)+weights(j)*fThetaPhi(j,k)*YLP(m+1,j,k);
    
                        flmM(l,m) = flmM(l,m)+weights(j)*fThetaPhi(j,k)*YLM(m,j,k);
                    else
                        flmP(l,m+1) = flmP(l,m+1)+weights(j)*fThetaPhi(j,k)*YLP(m+1,j,k);
                    end
                end
            end
        end 
    end

    flmP = sqrt(2*pi)/(2*b)*flmP;
    flmM = sqrt(2*pi)/(2*b)*flmM;
end

