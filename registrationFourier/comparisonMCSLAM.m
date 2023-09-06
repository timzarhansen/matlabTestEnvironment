clc
clear
currentNumber = 21;

motionCompensationImage = readmatrix("csvFiles/IROSResults/imagesComparison/motionCompensation"+string(currentNumber) + ".csv");
slamCompensationImage = readmatrix("csvFiles/IROSResults/imagesComparison/slamCompensation"+string(currentNumber) + ".csv");

N=sqrt(size(motionCompensationImage,1));


for j =1:N
    for i =1:N

        motionCompensationImagePlot(i,j) = motionCompensationImage((i-1)*N+j);
        slamCompensationImagePlot(i,j) = slamCompensationImage((i-1)*N+j);


    end
end
figure(1)
imagesc(motionCompensationImagePlot)

axis equal
axis image

figure(2)

imagesc(slamCompensationImagePlot)

axis equal
axis image






