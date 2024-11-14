function [outputArg1,outputArg2] = addCommandToBatchfile(Fullpath,Command,append)
%ADDCOMMANDTOBATCHFILE Summary of this function goes here
%   Detailed explanation goes here
if append
    FID = fopen(Fullpath,'a+');
else
    FID = fopen(Fullpath,'w');
end


fprintf(FID,'%s \n',Command);
fclose(FID);




end

