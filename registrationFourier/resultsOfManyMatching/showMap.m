clc
clear


%map = readmatrix("/home/tim-external/dataFolder/ValentinBunkerData/randomTests/scanNumber_0/0intensity256.csv");
map = readmatrix("completeMapTest.csv");


N=sqrt(size(map,1));
map = flip(map,2);

f = figure(4)
imagesc(map)

%legend('GICP', 'Super4PCS',  'NDT D2D 2D',  'NDT P2D', 'Our 2D FMS 128', 'Our Global FMS 2D 128','location','northwest', 'Interpreter', 'latex')
ylabel("120 m", 'Interpreter', 'latex')
xlabel("120 m", 'Interpreter', 'latex')

% ax = gca;
% ax.XLim = [0 125];

f.Position = [100 100 600 600];
box on

% nameOfMap = "OurGlobal256Map";
nameOfMap = "ValentinourGlobal256map";
% grid on
% nameOfFile = "/home/tim-external/Documents/icra2023FMS/figures/simulationMapsDifferentTechniques/"+nameOfMap;
nameOfFile = "/home/tim-external/Documents/icra2023FMS/figures/bunkerMapsDifferentTechniques/"+nameOfMap;

saveas(gcf,nameOfFile , 'pdf');
systemCommand = "pdfcrop " + nameOfFile +".pdf "+ nameOfFile+".pdf ";
system(systemCommand);

%%
% map = readmatrix("/home/tim-external/dataFolder/ValentinBunkerData/randomTests/scanNumber_0/0intensityShifted256.csv");
% N=sqrt(size(map,1));
% 
% 
% figure(5)
% imagesc(map)