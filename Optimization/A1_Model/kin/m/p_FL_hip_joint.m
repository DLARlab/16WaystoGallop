function [output1] = p_FL_hip_joint(var1)
    if coder.target('MATLAB')
        [output1] = p_FL_hip_joint_mex(var1);
    else
        coder.cinclude('p_FL_hip_joint_src.h');
        
        output1 = zeros(3, 1);

        
        coder.ceval('p_FL_hip_joint_src' ...
            ,coder.wref(output1) ...
            ,coder.rref(var1) );
    end
end
