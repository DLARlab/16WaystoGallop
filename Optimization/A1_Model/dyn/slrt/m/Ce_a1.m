function [output1] = Ce_a1(var1,var2)
    if coder.target('MATLAB')
        [output1] = Ce_a1_mex(var1,var2);
    else
        coder.cinclude('Ce_a1_src.h');
        
        output1 = zeros(18, 18);

        
        coder.ceval('Ce_a1_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) ,coder.rref(var2) );
    end
end
