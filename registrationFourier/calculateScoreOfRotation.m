function [COutput] = calculateScoreOfRotation(yawToTest,flm1,flm2,B) %#codegen


%NTest = 10;
%yawTMP=linspace(-0.5,0.5,NTest);
%COutput = zeros(NTest,1);
COutput = 0;
roll = 0;
pitch = 0;

for l = 1:B%was 2*B
    for m1 = 1:(l+1)
        for m2 = 1:(l+1)
            COutput = COutput + flm1(l,m1)*flm2(l,m2)*(-1)^(m1-m2)*exp(-1i*(m1-1)*roll)*wignerdFunction(pitch,l,(m1-1),(m2-1))*exp(-1i*(m2-1)*yawToTest);
            if isnan(COutput)
                COutput;
            end
        end
    end
end

display("done");
display(yawToTest);
display(COutput);
%COutput
end

