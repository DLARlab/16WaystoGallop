function [output1] = Jp_FRfoot(var1)
    if coder.target('MATLAB')
        [output1] = Jp_FRfoot_mex(var1);
    else
        coder.cinclude('Jp_FRfoot_src.h');
        
        output1 = zeros(3, 18);

        
        coder.ceval('Jp_FRfoot_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
