clc
clear
%interesting


from = 0;
to = 257;
resultingYawDiff = zeros(size(from:to,2),1);
translationDiff = zeros(size(from:to,2),2);




for indexCurrentFrame = from:to
    whichKeyframe = indexCurrentFrame
    nameOfCurrentTestingSet = 'gazeboCorrectedEvenAnglesPCLs_2_75';
    %nameOfFolder = '/home/tim-linux/dataFolder/gazeboCorrectedEvenAnglesPCLs/';
    %nameOfFolder = '/home/tim-linux/dataFolder/gazeboCorrectedUnevenAnglesPCLs/';

    %nameOfFolder = '/home/tim-linux/dataFolder/newStPereDatasetCorrectionOnly/';

    nameOfFolder = ['/home/tim-linux/dataFolder/' nameOfCurrentTestingSet '/'];
    firstScan=['pclKeyFrame',num2str(whichKeyframe),'.pcd'];
    secondScan =['pclKeyFrame',num2str(whichKeyframe+1),'.pcd'];
    
%% calculate initial guess

    
    
    transformation1 = readmatrix([nameOfFolder , 'position',num2str(whichKeyframe),'.csv']); 
    transformation2 = readmatrix([nameOfFolder , 'position',num2str(whichKeyframe+1),'.csv']);
    translation1=transformation1(1:3);
    translation2=transformation2(1:3);
    rotation1 = reshape(transformation1(4:end),[3,3]);
    rotation2 = reshape(transformation2(4:end),[3,3]);
    resultingTranslationDiff = -transformation1(1:3);
    resultingRotationDiff = transformation2(4:6)-transformation1(4:6);
    
    completeTransformation1 = eye(4);
    completeTransformation2 = eye(4);
    
    completeTransformation1(1:3,1:3)=rotation1;
    completeTransformation2(1:3,1:3)=rotation2;
    completeTransformation1(1,4)=transformation1(1);
    completeTransformation1(2,4)=transformation1(2);
    completeTransformation2(1,4)=transformation2(1);
    completeTransformation2(2,4)=transformation2(2);
    
    GTTransformationDiff = inv(completeTransformation1) *completeTransformation2;

    testTransformation = eye(4);
    testTransformation(1:3,1:3) = rotationMatrix(pi,0,0);
    %testTransformation(1:3,1:3) = rotationMatrix(0,0,pi);%*rotationMatrix(pi,0,0);
    resultingGTTransformation = testTransformation*inv(GTTransformationDiff);

    rpyGT = anglesR(resultingGTTransformation(1:3,1:3),'xyz')/180*pi;
    yawInitialGuess = rpyGT(3);





%%



    command =['rosrun underwaterslam registrationOfTwoPCLICP ',nameOfFolder, firstScan,' ', nameOfFolder, secondScan, ' ', num2str(yawInitialGuess)];
    %system(cmdStr);
    system(command);
    
    %%
    
    set(groot,'defaultAxesTickLabelInterpreter','latex');     
    

    
   

    
    
    figure(9)
    clf
    
    
    pointCloudResult1 = pcread(['csvFiles/resulting0PCL1.pcd']);
    pointCloudResult2 = pcread(['csvFiles/resulting0PCL2.pcd']);
    tform = eye(4);
    tform(1:3,1:3) = rotationMatrix(0,0,0);
    tform(1,4)=0;%x value
    tform(2,4)=0;%y value
    tform=affine3d(tform');
    pointCloudResult1 = pctransform(pointCloudResult1,tform);
    
    
    
    pcshowpair(pointCloudResult1, pointCloudResult2)
    
    
    view(90,-90)
    set(gca,'color','w', 'XColor','k', 'YColor','k');
    set(gcf,'color','w');
        
    %     xlabel('X-Axis in m', 'Interpreter', 'latex')
    %     ylabel('Y-Axis in m', 'Interpreter', 'latex')
        
    
    %%
    

    
    GTTransformationDiff = inv(completeTransformation1) *completeTransformation2;
    rpyTMP = anglesR(GTTransformationDiff(1:3,1:3),'xyz')/180*pi;

    
    
    
    testTransformation = eye(4);
    testTransformation(1:3,1:3) = rotationMatrix(pi,0,0);
    %testTransformation(1:3,1:3) =
    %rotationMatrix(0,0,pi);%*rotationMatrix(pi,0,0);
    resultingGTTransformation = testTransformation*inv(GTTransformationDiff);

    resultingTransformationOfScan = readmatrix(['csvFiles/resultingTransformation0.csv']);
    resultingYawDiff(indexCurrentFrame-from+1) = atan2(sin(rpyTMP(3) - resultingTransformationOfScan(6)), cos(rpyTMP(3) - resultingTransformationOfScan(6)))
    %atan2(sin(rpyTMP(3) - resultingTransformationOfScan(6)), cos(rpyTMP(3) - resultingTransformationOfScan(6)))
    translationDiff(indexCurrentFrame-from+1,:) = resultingTransformationOfScan(1:2)-resultingGTTransformation(1:2,4)
    save([ 'resultsOfManyMatching/' nameOfCurrentTestingSet 'ICP.mat'],'resultingYawDiff','translationDiff')
end

%%
% figure(10)
% plot(resultingYawDiffBestMatching)
% 
% %translationDiff
% figure(11)
% plot(vecnorm(translationDiffBestMatching'))


%










