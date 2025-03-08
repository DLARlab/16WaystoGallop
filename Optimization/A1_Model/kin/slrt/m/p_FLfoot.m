function [output1] = p_FLfoot(var1)
    if coder.target('MATLAB')
        [output1] = p_FLfoot_mex(var1);
    else
        coder.cinclude('p_FLfoot_src.h');
        
        output1 = zeros(3, 1);

        
        coder.ceval('p_FLfoot_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
