function [output1] = Jp_RRfoot(var1)
    if coder.target('MATLAB')
        [output1] = Jp_RRfoot_mex(var1);
    else
        coder.cinclude('Jp_RRfoot_src.h');
        
        output1 = zeros(3, 18);

        
        coder.ceval('Jp_RRfoot_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
