function [output1] = De_a1(var1)
    if coder.target('MATLAB')
        [output1] = De_a1_mex(var1);
    else
        coder.cinclude('De_a1_src.h');
        
        output1 = zeros(18, 18);

        
        coder.ceval('De_a1_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
