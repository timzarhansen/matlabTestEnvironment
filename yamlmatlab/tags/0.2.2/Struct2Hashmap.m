function ArrList = Struct2Hashmap(S)
% This function converts matla struct to java object of type
% java.util.HashMap.
% Inputs:
%   - S   : matlab struct
% Outputs:
%   - hmap: java.util.HashMap object

%======================================================================
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
        jc    07-Mar-11   Support for date time and cell arrays nxm
%}
%======================================================================

if ((~isstruct(S)))
    error('struct2hashmap:invalid','%s',...
        'struct2hashmap only accepts structures');
end
ArrList = java.util.ArrayList;

for n=1:numel(S)
    
    hmap = java.util.LinkedHashMap;
    for fn = fieldnames(S)'
        % fn iterates through the field names of S
        % fn is a 1x1 cell array
        
        val = getfield(S(n),fn{1});
        if isstruct(val)
            val=Struct2Hashmap(val);
        end
        
        %todo check matrices, and cells
        vn = java.util.ArrayList();
        if not(isscalar(val)) && not(ischar(val)) && not(isa(val,'java.util.LinkedHashMap')) || isa(val,'java.util.ArrayList')
            if not(isscalar(val)) && isnumeric(val) % numeric
                if size(val,1)==1 % one row
                    arrayfun(@(x)vn.add(x),val);
                else
                    for i=1:size(val,1)
                        vnr = java.util.ArrayList();
                        arrayfun(@(x)vnr.add(x),val(i,:));
                        vn.add(vnr);
                    end
                end
            elseif iscell(val)
                if size(val,1)==1 % one row                    
                    cellfun(@(x)vn.add(JavaObjType(x)),val);
                else
                    for i=1:size(val,1)
                        vnr = java.util.ArrayList();
                        cellfun(@(x)vnr.add(JavaObjType(x)),val(i,:));
                        vn.add(vnr);
                    end
                end              
                
            else
                error('Unknown data type');
            end
            val = vn;
        end
        
        hmap.put(fn{1},JavaObjType(val));
    end
    if numel(S)>1
        ArrList.add(hmap);
    else
        ArrList=hmap;
    end
end
end
function outDataFormat=JavaObjType(x)
if isa(x,'DateTime')
    outDataFormat=java.util.Date(datestr(x));    
elseif ischar(x)
    outDataFormat = EncodeUnicode(x);
else
    outDataFormat=x;
end
end