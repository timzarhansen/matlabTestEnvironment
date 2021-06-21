function [data] = readJsonGraphPCL(nameOfFile)
    % load in a Jason file of graph-based defined formulation
    % in data it should be data(keyframe,pointsInPCL,Pos) while the first
    % pos is the position of the PCL(already rotated to zero)

    fid = fopen(nameOfFile); 
    raw = fread(fid,inf); 
    str = char(raw'); 
    fclose(fid); 
    val = jsondecode(str);
    for i =1:length(val.keyFrames)
        data(i,1,1:3) = [val.keyFrames(i).position.x , val.keyFrames(i).position.y , val.keyFrames(i).position.z];
        % rotation still necessary
        eulerAngles = [val.keyFrames(i).position.yaw , val.keyFrames(i).position.pitch , val.keyFrames(i).position.roll];
        
        yawRotation = [cos(eulerAngles(1)), -sin(eulerAngles(1)), 0;sin(eulerAngles(1)), cos(eulerAngles(1)) ,0;0,0,1];
        pitchRotation = [cos(eulerAngles(2)) , 0 , sin(eulerAngles(2)); 0 , 1 , 0 ; -sin(eulerAngles(2)),0,cos(eulerAngles(2))];
        rollRotation = [1,0,0;0,cos(eulerAngles(3)), -sin(eulerAngles(3));0,sin(eulerAngles(3)),cos(eulerAngles(3))];
        
        rotm =yawRotation*pitchRotation*rollRotation;
        for j = 1:length(val.keyFrames(i).pointCloud)
            data(i,j+1,1:3) = rotm*[val.keyFrames(i).pointCloud(j).point.x ; val.keyFrames(i).pointCloud(j).point.y ; val.keyFrames(i).pointCloud(j).point.z];
        end
    end
end

