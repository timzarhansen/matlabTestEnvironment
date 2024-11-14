clc
clear

currentMapMatrix = readmatrix("csvFiles/current3DMapTest.csv");



N=nthroot(size(currentMapMatrix,1),3);
currentMap3D =zeros(N,N,N);




for j =1:N
    for i =1:N
        for k =1:N
            currentMap3D(i,j,k) = currentMapMatrix((i-1)*N+j+(k-1)*N*N);
        end
    end
end


volumeViewer(currentMap3D)



%%
globalVector = [10,20,2,1]';

A12 = transformationsMatrix(0,0,0,1,2,3);
A23 = transformationsMatrix(0.2,0.3,0.5,0,0,0);
A34 = transformationsMatrix(0,0,0,5,3,1);
A45 = transformationsMatrix(0.1,0.2,0.5,0,0,0);

B12 = transformationsMatrix(0.2,0.3,0.5,1,2,3);
B23 = transformationsMatrix(0.1,0.2,0.5,5,3,1);
localVector = A45*A34*A23*A12*globalVector
localVectorB = B23*B12*globalVector

globalVector1 = inv(A12)*inv(A23)*inv(A34)*inv(A45)*localVector
globalVector2 = inv(B12)*inv(B23)*localVectorB

%%
transofrmationTest = transformationsMatrix(0,0,-pi/2,-4,6,0)





inv(transofrmationTest)
inv(transofrmationTest)*[-5,2,-12,1]'










