function Data = Hash2Struct(hashMap)
% This function converts object of type java.util.HashMap to matlab struct.
% Inputs:
%   - hashMap : java.util.HashMap object
% Outputs:
%   - Data    : matlab struct
% This code is based on template provided by Jan Siroky on 
% http://stackoverflow.com/questions/1638018/is-there-already-a-yaml-library-parser-for-matlab


%  %======================================================================
%{
		Copyright (c) 2011
		This program is a result of a joined cooperation of Energocentrum
		PLUS, s.r.o. and Czech Technical University (CTU) in Prague.
        The program is maintained by Energocentrum PLUS, s.r.o. and
        licensed under the terms of MIT license. Full text of the license
        is included in the program release.
		
        Author(s):
		Jiri Cigler, Dept. of Control Engineering, CTU Prague
		Jan  Siroky, Energocentrum PLUS s.r.o.
		
        Implementation and Revisions:

        Auth  Date        Description of change
        ----  ---------   --------------------------------------------------
        jc    01-Mar-11   First implementation
%}
%======================================================================

Data = [];

iterator = hashMap.keySet().iterator();
while (iterator.hasNext())
    field = iterator.next();
    if ~isempty(field)
        d =  hashMap.get(field);
        switch class(d)
            case {'java.util.LinkedHashMap'}
                Data.(field) = Hash2Struct(d);
            case {'java.util.ArrayList'}
                switch class(d.get(0))
                    case {'char' ,'double'}
                        it = d.iterator();
                        val={};
                        while (it.hasNext())
                            val(end+1)={it.next()};
                        end
                        if all(cellfun(@(x) isnumeric(x),val))
                            val = cell2mat(val);
                        end
                    case {'java.util.ArrayList'}
                        itr = d.iterator();
                        val={};
                        r=1;
                        while (itr.hasNext())
                            row=itr.next();
                            itc = row.iterator();
                            c=1;
                            while(itc.hasNext())
                                val{r,c}=itc.next();
                                c=c+1;
                            end
                            r=r+1;
                        end
                        if all(cellfun(@(x) isnumeric(x),val))
                            val = cell2mat(val);
                        end
                        
                    otherwise
                        error('unknown  java datatype');
                end
                Data.(field) = val;
            otherwise
                Data.(field) = d;
        end
    end
end