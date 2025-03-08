function [output1] = Ge_a1(var1)
    if coder.target('MATLAB')
        [output1] = Ge_a1_mex(var1);
    else
        coder.cinclude('Ge_a1_src.h');
        
        output1 = zeros(18, 1);

        
        coder.ceval('Ge_a1_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
