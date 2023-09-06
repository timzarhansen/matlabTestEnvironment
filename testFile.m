clc
clear
figure(8)
clf

correlationMatrixShift1D = readmatrix("csvFiles/resultingCorrelationShift.csv");
resultSize = nthroot(length(correlationMatrixShift1D),2);
%A = reshape(results,resultSize,resultSize);
correlationMatrixShift2D =zeros(resultSize);
for j = 1:resultSize
    for i =1:resultSize
            correlationMatrixShift2D(i,j) = correlationMatrixShift1D((i-1)*resultSize+j);
            correlationMatrixShift2D(i,j) = correlationMatrixShift1D((i-1)*resultSize+j);
    end
end
testImage = correlationMatrixShift2D;




[Xplot,Yplot]=meshgrid(1:resultSize,1:resultSize);
surf(Xplot,Yplot,(testImage),'edgecolor', 'none');
xlabel("x-axis");
ylabel("y-axis");


simpleCopy = correlationMatrixShift2D;
% p=Fast2DPeakFind(correlationMatrixShift2D);
radius = resultSize/25;
p = [343,318];
% p = [307,330];
hold on 
heightPeak = correlationMatrixShift2D(p(2),p(1));
areaPeakMaximum = heightPeak*pi*radius*radius;
integralRadius = 0;
for k = -ceil(radius):ceil(radius)
    for l = -ceil(radius):ceil(radius)
        if sqrt(k^2+l^2)<radius
            integralRadius = integralRadius+correlationMatrixShift2D(p(2)+l,p(1)+k);
        end
    end
end
integralRadius/areaPeakMaximum


for k = 1:size(p(1:2:end),1)
%     display(k)
    zForPlot(k)=correlationMatrixShift2D(p(2*k),p(2*k-1));

end
plot3(p(1:2:end),p(2:2:end),zForPlot,'r+')
% view(0,0)