function [output1] = R_RR_thigh_joint(var1)
    if coder.target('MATLAB')
        [output1] = R_RR_thigh_joint_mex(var1);
    else
        coder.cinclude('R_RR_thigh_joint_src.h');
        
        output1 = zeros(3, 3);

        
        coder.ceval('R_RR_thigh_joint_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
