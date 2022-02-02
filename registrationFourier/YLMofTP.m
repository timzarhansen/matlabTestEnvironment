function [YlP, YlM] = YLMofTP(l,theta,phi)
    YlP=zeros(l+1,size(theta,2),size(phi,2));
    YlM=zeros(l,size(theta,2),size(phi,2));
%     Plm = legendre(l,cos(theta));

    for m = 0:l
        syms x;
        % get symbolic form of Legendre function P_l(x)
        leg = legendreP(l,x);
        % differentiate it m times
        legDiff = diff(leg,x,abs(m));
        % calculate associated legendre function P_lm(x)
        plm = ((-1)^abs(m))*((1 - x^2)^(abs(m)/2))*legDiff;
        % calc solution
        x0=cos(theta);
        plmSolutionPlus = double(subs(plm,x,x0));

        if m>0
            plmSolutionMinus = plmSolutionPlus*(-1)^abs(m)*factorial(l-abs(m))/factorial(l+abs(m));
        


            a = (2*l+1)*factorial(l-m);
            b = 4*pi*factorial(l+m);
            C = sqrt(a/b);
            YlP(m+1,:,:) = (-1)^m .* C .*plmSolutionPlus' .*exp(-1i*m*phi);

            a = (2*l+1)*factorial(l+m);
            b = 4*pi*factorial(l-m);
            C = sqrt(a/b);
            YlM(m,:,:) = (-1)^(-m) .* C .*plmSolutionMinus' .*exp(1i*m*phi);

        else

            a = (2*l+1)*factorial(l-m);
            b = 4*pi*factorial(l+m);
            C = sqrt(a/b);
            YlPTMP = (-1)^m .* C.*plmSolutionPlus'.*exp(-1i*m*phi);
            YlP(m+1,:,:) = reshape(YlPTMP,1,size(theta,2),size(phi,2));

        end


    end

end

