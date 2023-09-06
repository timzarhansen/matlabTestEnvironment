clc
clear

voxelDataUsed1 = readmatrix("csvFiles/voxelDataFFTW1.csv");
voxelDataUsed2 = readmatrix("csvFiles/voxelDataFFTW2.csv");
N=256;
voxelData1 =zeros(N,N);
voxelData2 =zeros(N,N);

for j =1:N
    for k =1:N
        voxelData1(k,j) = voxelDataUsed1((k-1)*N+j);
        voxelData2(k,j) = voxelDataUsed2((k-1)*N+j);
    end
end










A = imread('csvFiles/testImage1.png');
B = imread('csvFiles/testImage2.png');



% A = voxelData1;
% B = voxelData2;



% B = imread('csvFiles/testImageNew1.png');
% A = imread('csvFiles/testImageNew1.png');

B=B(:,:,3);
maximumB = max(max(B));
B = maximumB- B;
A=A(:,:,3);
maximumA = max(max(A));
A = maximumA- A;






B = double(B);
A = double(A);

B = imgaussfilt(B,2);
A = imgaussfilt(A,2);

figure(1)
imagesc(A)
figure(2)

imagesc(B)



maximumOverall = max([max(max(A)), max(max(B))]);
normalizedA = A/maximumOverall;
normalizedB = B/maximumOverall;
figure(3)
sizeTMP = size(normalizedA,1);
C_New = (xcorr2(normalizedA,normalizedB));

% C_New = C_New(sizeTMP/2:(size(C_New,1)-(sizeTMP/2-1)),sizeTMP/2:(size(C_New,1)-(sizeTMP/2-1)));
% imagesc(C_New)
[Xplot,Yplot] = meshgrid(1:size(C_New,1),1:size(C_New,1));
surf(Xplot,Yplot,(C_New));
xlabel("x-axis");
ylabel("y-axis");
view(0,0)

figure(4)

C_Simple = xcorr2(ones(size(normalizedA)),ones(size(normalizedB)));
% C_Simple = C_Simple(sizeTMP/2:(size(C_Simple,1)-(sizeTMP/2-1)),sizeTMP/2:(size(C_Simple,1)-(sizeTMP/2-1)));

% C_Simple = sqrt(C_Simple);
[Xplot,Yplot] = meshgrid(1:size(C_Simple,1),1:size(C_Simple,1));
surf(Xplot,Yplot,((C_Simple)),'EdgeColor','none');
xlabel("x-axis");
ylabel("y-axis");

figure(5)
clf

% figure(15)

resultingCorrectedCorrelation = (1./C_Simple).*C_New;
[Xplot,Yplot] = meshgrid(1:size(resultingCorrectedCorrelation,1),1:size(resultingCorrectedCorrelation,1));
surf(Xplot,Yplot,(resultingCorrectedCorrelation));
xlabel("x-axis");
ylabel("y-axis");



%% Find Peaks Maybe
simpleCopy = resultingCorrectedCorrelation;
p=Fast2DPeakFind(resultingCorrectedCorrelation);
hold on 

for k = 1:size(p(1:2:end),1)
%     display(k)
    zForPlot(k)=resultingCorrectedCorrelation(p(2*k),p(2*k-1));

end
plot3(p(1:2:end),p(2:2:end),zForPlot,'r+')
view(0,0)