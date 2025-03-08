function [output1] = Jp_FLfoot(var1)
    if coder.target('MATLAB')
        [output1] = Jp_FLfoot_mex(var1);
    else
        coder.cinclude('Jp_FLfoot_src.h');
        
        output1 = zeros(3, 18);

        
        coder.ceval('Jp_FLfoot_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
