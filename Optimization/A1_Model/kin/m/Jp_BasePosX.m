function [output1] = Jp_BasePosX(var1)
    if coder.target('MATLAB')
        [output1] = Jp_BasePosX_mex(var1);
    else
        coder.cinclude('Jp_BasePosX_src.h');
        
        output1 = zeros(3, 18);

        
        coder.ceval('Jp_BasePosX_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
