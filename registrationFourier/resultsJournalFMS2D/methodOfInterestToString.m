function [stringOut] = methodOfInterestToString(whichMethod)
%METHODOFINTERESTTOSTRING Summary of this function goes here
%   Detailed explanation goes here

% 1: GICP, 2: SUPER4PCS, 3: NDT D2D 2D, 4: NDT P2D, 5: FourierMellinTransform,
% 6: Our FMS 2D 7: FMS hamming 8: FMS none
% 9: Feature0 10: Feature1  11: Feature2 12: Feature3 13: Feature4 14: Feature5
% 15: gmmRegistrationD2D 16: gmmRegistrationP2D 

switch whichMethod
    case 1
        stringOut = "GICP";
    case 2
        stringOut = "SUPER4PCS";
    case 3
        stringOut = "NDT D2D 2D";
    case 4
        stringOut = "NDT P2D";
    case 5
        stringOut = "FourierMellinTransform";
    case 6
        stringOut = "Our FMS 2D";
    case 7
        stringOut = "FMS hamming";
    case 8
        stringOut = "FMS none";
    case 9
        stringOut = "Feature AKAZE";
    case 10
        stringOut = "Feature KAZE";
    case 11
        stringOut = "Feature ORB";
    case 12
        stringOut = "Feature BRISK";
    case 13
        stringOut = "Feature SURF";
    case 14
        stringOut = "Feature SIFT ";
    case 15
        stringOut = "gmmRegistrationD2D";
    case 16
        stringOut = "gmmRegistrationP2D";

end

