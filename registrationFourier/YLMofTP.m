function [Yl] = YLMofTP(l,theta,phi)


    Plm = legendre(l,cos(theta));

    for m = 0:l
        a = (2*l+1)*factorial(l-m);
        b = 4*pi*factorial(l+m);
        C = sqrt(a/b);
        Yl(m+1) = (-1)^m * C .*Plm(m+1) .*exp(1i*m*phi);
    end

end

