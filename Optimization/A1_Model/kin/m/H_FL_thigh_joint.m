function [output1] = H_FL_thigh_joint(var1)
    if coder.target('MATLAB')
        [output1] = H_FL_thigh_joint_mex(var1);
    else
        coder.cinclude('H_FL_thigh_joint_src.h');
        
        output1 = zeros(4, 4);

        
        coder.ceval('H_FL_thigh_joint_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
