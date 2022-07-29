clc
clear
%fileName = "resultsOfManyMatching/gazeboCorrectedEvenAnglesPCLs_2_7564.mat";
%fileName = "resultsOfManyMatching/gazeboCorrectedEvenAnglesPCLs_2_75.mat";
fileName = "resultsOfManyMatching/gazeboCorrectedEvenAnglesPCLs_2_75ICP.mat";
%fileName = "resultsOfManyMatching/gazeboCorrectedEvenAnglesPCLs_4_100.mat"
%fileName = "resultsOfManyMatching/gazeboCorrectedUnevenAnglesPCLs_2_75.mat"
%fileName = "resultsOfManyMatching/gazeboCorrectedUnevenAnglesPCLs_4_100.mat"
load(fileName)

load("resultsOfManyMatching/gazeboCorrectedEvenAnglesPCLs_2_7564.mat")
%%

figure(10)
clf
hold on
set(gca,'DefaultLineLineWidth',2)
p1 = plot(resultingYawDiffBestMatching(1:find(resultingYawDiffBestMatching, 1, 'last') )*180/pi)
p2 = plot(resultingYawDiffInitialGuess(1:find(resultingYawDiffInitialGuess, 1, 'last') )*180/pi)
p3 = plot(resultingYawDiff(1:find(resultingYawDiff, 1, 'last') )*180/pi)
p1.Color(4) = 0.75;
p2.Color(4) = 0.75;
p3.Color(4) = 0.75;
xlabel("Scan Number", 'Interpreter', 'latex');
ylabel("Angle in Degree", 'Interpreter', 'latex');
pbaspect([2 1 1])

leg1 = legend('Best Match FMS','Initial Guess FMS', 'Initial Guess ICP');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);


saveas(gcf, 'outputPDFs/multipleScansMatchingAngleDiff', 'pdf')
system('pdfcrop outputPDFs/multipleScansMatchingAngleDiff.pdf outputPDFs/multipleScansMatchingAngleDiff.pdf');





figure(11)
clf
hold on
set(gca,'DefaultLineLineWidth',2)
p1 = plot(vecnorm(translationDiffBestMatching(1:find(resultingYawDiffBestMatching, 1, 'last'),:)'))
p2 = plot(vecnorm(translationDiffInitialGuess(1:find(resultingYawDiffInitialGuess, 1, 'last'),:)'))
p3 = plot(vecnorm(translationDiff(1:81,:)'))
xlabel("Scan Number", 'Interpreter', 'latex');
ylabel("Euclidean distance in m", 'Interpreter', 'latex');
p1.Color(4) = 0.75;
p2.Color(4) = 0.75;
p3.Color(4) = 0.75;
pbaspect([2 1 1])

leg1 = legend('Best Match FMS','Initial Guess FMS', 'Initial Guess ICP');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',10);


saveas(gcf, 'outputPDFs/multipleScansMatchingTranslationDiff', 'pdf')
system('pdfcrop outputPDFs/multipleScansMatchingTranslationDiff.pdf outputPDFs/multipleScansMatchingTranslationDiff.pdf');


meanInitialGuessAngle = mean(abs(resultingYawDiffInitialGuess(1:find(resultingYawDiffBestMatching, 1, 'last') )*180/pi))
stdInitialGuessAngle = std(abs(resultingYawDiffInitialGuess(1:find(resultingYawDiffBestMatching, 1, 'last') )*180/pi))


meanBestGuessAngle = mean(abs(resultingYawDiffBestMatching(1:find(resultingYawDiffBestMatching, 1, 'last') )*180/pi))
stdBestGuessAngle = std(abs(resultingYawDiffBestMatching(1:find(resultingYawDiffBestMatching, 1, 'last') )*180/pi))


meanBestGuessTranslation = mean(vecnorm(translationDiffBestMatching(1:find(resultingYawDiffBestMatching, 1, 'last'),:)'))
stdBestGuessTranslation = std(vecnorm(translationDiffBestMatching(1:find(resultingYawDiffBestMatching, 1, 'last'),:)'))


meanInitialGuessTranslation = mean(vecnorm(translationDiffInitialGuess(1:find(resultingYawDiffInitialGuess, 1, 'last'),:)'))
stdInitialGuessTranslation  = std(vecnorm(translationDiffInitialGuess(1:find(resultingYawDiffInitialGuess, 1, 'last'),:)'))


meanICPAngle = mean(abs(resultingYawDiff(1:find(resultingYawDiff, 1, 'last') )*180/pi))
stdICPAngle = std(abs(resultingYawDiff(1:find(resultingYawDiff, 1, 'last') )*180/pi))


meanICPTranslation = mean(vecnorm(translationDiff(1:81,:)'))
stdICPTranslation = std(vecnorm(translationDiff(1:81,:)'))







