function [output1] = Be_a1(var1)
    if coder.target('MATLAB')
        [output1] = Be_a1_mex(var1);
    else
        coder.cinclude('Be_a1_src.h');
        
        output1 = zeros(18, 12);

        
        coder.ceval('Be_a1_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
