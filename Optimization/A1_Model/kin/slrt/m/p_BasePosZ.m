function [output1] = p_BasePosZ(var1)
    if coder.target('MATLAB')
        [output1] = p_BasePosZ_mex(var1);
    else
        coder.cinclude('p_BasePosZ_src.h');
        
        output1 = zeros(3, 1);

        
        coder.ceval('p_BasePosZ_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
