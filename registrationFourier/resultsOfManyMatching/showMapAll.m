clc
clear
%1: our 256 2:GICP 3:super 4: NDT d2d 5: NDT P2D 6: our global 256
WHICH_METHOD_USED  = 6;
for index = 3:54
%     map = readmatrix("/home/tim-external/dataFolder/SimulationEnvironment/experimentForVideoICRA/consecutiveFromMiddle/consecutiveScanCreation_"+WHICH_METHOD_USED+"_"+index+".csv");
%     map = readmatrix("/home/tim-external/dataFolder/SimulationEnvironment/experimentForVideoICRA/consecutiveAll/consecutiveScanCreation_"+WHICH_METHOD_USED+"_"+index+".csv");
    map = readmatrix("/home/tim-external/dataFolder/SimulationEnvironment/experimentForVideoICRA/scanNumber"+index+".csv");
    
    N=256;
    voxelData =zeros(N,N);
    for j =1:N
        for k =1:N
            voxelData(k,j) = map((k-1)*N+j);
        end
    end

    map = voxelData;

    N=sqrt(size(map,1));
    map = flip(map,2);
    map = rot90(rot90(rot90(map)));
    
    f = figure(4)
    imagesc(map)
    
    %legend('GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D', 'Our 2D FMS 128', 'Our Global FMS 2D 128','location','northwest', 'Interpreter', 'latex')
    ylabel("120 m", 'Interpreter', 'latex')
    xlabel("120 m", 'Interpreter', 'latex')
    
    % ax = gca;
    % ax.XLim = [0 125];
    
    f.Position = [500 600 600 600];
    box on
    pause(0.5)
end




