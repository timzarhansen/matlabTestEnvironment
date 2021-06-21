function [d] = wignerdFunction(pitch,l,m1,m2)
%WIGNERDFUNCTION Summary of this function goes here

mu = abs(m1-m2);
vi = abs(m1+m2);
s = l-(mu+vi)/2;
if m2>=m1
    zeta = 1;
else
    zeta = (-1)^(m2-m1);
end
d = zeta*sqrt(factorial(s)*factorial(s+mu+vi)/factorial(s+mu)/factorial(s+vi))*(sin(pitch/2))^(mu)*cos(pitch/2)^vi * jacobiP(s,mu,vi,cos(pitch));



%d=sqrt(factorial(l+m2)/factorial(l+m1)*factorial(l-m2)/factorial(l-m1))*sin(pitch/2)^(sign(m2-m1))*cos(pitch/2)^(sign(m1+m2))*jacobiP(l-m2,m2-m1,m1+m2,cos(pitch));
end

