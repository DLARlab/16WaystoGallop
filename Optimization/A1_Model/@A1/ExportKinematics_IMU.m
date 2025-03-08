function ExportKinematics_IMU(obj, export_function, export_path)
% Computes the Forward Kinematics to be used for state estimation (IMU to contact)
%
%   Author: Ross Hartley
%   Date:   11/15/2017
%
% Encoder Vector
encoders = SymVariable(obj.States.x(7:end));

% Frames
body_frame = CoordinateFrame(...
    'Name','Body',...
    'Reference',obj.Joints(getJointIndices(obj,'BaseRotX')),...
    'Offset',[0, 0, 0],...
    'R',[0, 0, 0]);

H_WB = body_frame.computeForwardKinematics;

H_WFR = obj.ContactPoints.FR_foot.computeForwardKinematics;
H_WFL = obj.ContactPoints.FL_foot.computeForwardKinematics;
H_WRR = obj.ContactPoints.RR_foot.computeForwardKinematics;
H_WRL = obj.ContactPoints.RL_foot.computeForwardKinematics;

H_WI = obj.OtherPoints.VectorNav.computeForwardKinematics;

H_I1B = H_WI\H_WB; H_I1B = subs(H_I1B, obj.States.x(1:6), zeros(6,1));
H_I1FR = H_WI\H_WFR; H_I1FR = subs(H_I1FR, obj.States.x(1:6), zeros(6,1));
H_I1FL = H_WI\H_WFL; H_I1FL = subs(H_I1FL, obj.States.x(1:6), zeros(6,1));

H_I1RR = H_WI\H_WRR; H_I1RR = subs(H_I1RR, obj.States.x(1:6), zeros(6,1));
H_I1RL = H_WI\H_WRL; H_I1RL = subs(H_I1RL, obj.States.x(1:6), zeros(6,1));

% front legs
H_FRFL = H_WFR\H_WFL; H_FRFL = subs(H_FRFL, obj.States.x(1:6), zeros(6,1));
H_FLFR = H_WFL\H_WFR; H_FLFR = subs(H_FLFR, obj.States.x(1:6), zeros(6,1));
% rear legs
H_RRRL = H_WRR\H_WRL; H_RRRL = subs(H_RRRL, obj.States.x(1:6), zeros(6,1));
H_RLRR = H_WRL\H_WRR; H_RLRR = subs(H_RLRR, obj.States.x(1:6), zeros(6,1));
% right legs
H_FRRR = H_WFR\H_WRR; H_FRRR = subs(H_FRRR, obj.States.x(1:6), zeros(6,1));
H_RRFR = H_WRR\H_WFR; H_RRFR = subs(H_RRFR, obj.States.x(1:6), zeros(6,1));
% left legs
H_FLRL = H_WFL\H_WRL; H_FLRL = subs(H_FLRL, obj.States.x(1:6), zeros(6,1));
H_RLFL = H_WRL\H_WFL; H_RLFL = subs(H_RLFL, obj.States.x(1:6), zeros(6,1));

% Export Functions
export_function(H_I1B, 'H_VectorNav_to_Pelvis', export_path, encoders);
export_function(H_I1B(1:3,1:3), 'R_VectorNav_to_Pelvis', export_path, encoders);
export_function(H_I1B(1:3,end), 'p_VectorNav_to_Pelvis', export_path, encoders);

export_function(H_I1FR, 'H_VectorNav_to_FRfoot', export_path, encoders);
export_function(H_I1FR(1:3,1:3), 'R_VectorNav_to_FRfoot', export_path, encoders);
export_function(H_I1FR(1:3,end), 'p_VectorNav_to_FRfoot', export_path, encoders);
export_function(jacobian(H_I1FR(1:3,end),encoders), 'Jp_VectorNav_to_FRfoot', export_path, encoders);

export_function(H_I1FL, 'H_VectorNav_to_FLfoot', export_path, encoders);
export_function(H_I1FL(1:3,1:3), 'R_VectorNav_to_FLfoot', export_path, encoders);
export_function(H_I1FL(1:3,end), 'p_VectorNav_to_FLfoot', export_path, encoders);
export_function(jacobian(H_I1FL(1:3,end),encoders), 'Jp_VectorNav_to_FLfoot', export_path, encoders);

export_function(H_I1RR, 'H_VectorNav_to_RRfoot', export_path, encoders);
export_function(H_I1RR(1:3,1:3), 'R_VectorNav_to_RRfoot', export_path, encoders);
export_function(H_I1RR(1:3,end), 'p_VectorNav_to_RRfoot', export_path, encoders);
export_function(jacobian(H_I1RR(1:3,end),encoders), 'Jp_VectorNav_to_RRfoot', export_path, encoders);

export_function(H_I1RL, 'H_VectorNav_to_RLfoot', export_path, encoders);
export_function(H_I1RL(1:3,1:3), 'R_VectorNav_to_RLfoot', export_path, encoders);
export_function(H_I1RL(1:3,end), 'p_VectorNav_to_RLfoot', export_path, encoders);
export_function(jacobian(H_I1RL(1:3,end),encoders), 'Jp_VectorNav_to_RLfoot', export_path, encoders);



export_function(H_FRFL, 'H_FRfoot_to_FLfoot', export_path, encoders);
export_function(H_FRFL(1:3,1:3), 'R_FRfoot_to_FLfoot', export_path, encoders);
export_function(H_FRFL(1:3,end), 'p_FRfoot_to_FLfoot', export_path, encoders);
export_function(jacobian(H_FRFL(1:3,end),encoders), 'Jp_FRfoot_to_FLfoot', export_path, encoders);

export_function(H_FLFR, 'H_FLfoot_to_FRfoot', export_path, encoders);
export_function(H_FLFR(1:3,1:3), 'R_FLfoot_to_FRfoot', export_path, encoders);
export_function(H_FLFR(1:3,end), 'p_FLfoot_to_FRfoot', export_path, encoders);
export_function(jacobian(H_FLFR(1:3,end),encoders), 'Jp_FLfoot_to_FRfoot', export_path, encoders);


export_function(H_RRRL, 'H_RRfoot_to_RLfoot', export_path, encoders);
export_function(H_RRRL(1:3,1:3), 'R_RRfoot_to_RLfoot', export_path, encoders);
export_function(H_RRRL(1:3,end), 'p_RRfoot_to_RLfoot', export_path, encoders);
export_function(jacobian(H_RRRL(1:3,end),encoders), 'Jp_RRfoot_to_RLfoot', export_path, encoders);

export_function(H_RLRR, 'H_RLfoot_to_RRfoot', export_path, encoders);
export_function(H_RLRR(1:3,1:3), 'R_RLfoot_to_RRfoot', export_path, encoders);
export_function(H_RLRR(1:3,end), 'p_RLfoot_to_RRfoot', export_path, encoders);
export_function(jacobian(H_RLRR(1:3,end),encoders), 'Jp_RLfoot_to_RRfoot', export_path, encoders);


export_function(H_FRRR, 'H_FRfoot_to_RRfoot', export_path, encoders);
export_function(H_FRRR(1:3,1:3), 'R_FRfoot_to_RRfoot', export_path, encoders);
export_function(H_FRRR(1:3,end), 'p_FRfoot_to_RRfoot', export_path, encoders);
export_function(jacobian(H_FRRR(1:3,end),encoders), 'Jp_FRfoot_to_RRfoot', export_path, encoders);

export_function(H_RRFR, 'H_RRfoot_to_FRfoot', export_path, encoders);
export_function(H_RRFR(1:3,1:3), 'R_RRfoot_to_FRfoot', export_path, encoders);
export_function(H_RRFR(1:3,end), 'p_RRfoot_to_FRfoot', export_path, encoders);
export_function(jacobian(H_RRFR(1:3,end),encoders), 'Jp_RRfoot_to_FRfoot', export_path, encoders);


export_function(H_FLRL, 'H_FLfoot_to_RLfoot', export_path, encoders);
export_function(H_FLRL(1:3,1:3), 'R_FLfoot_to_RLfoot', export_path, encoders);
export_function(H_FLRL(1:3,end), 'p_FLfoot_to_RLfoot', export_path, encoders);
export_function(jacobian(H_FLRL(1:3,end),encoders), 'Jp_FLfoot_to_RLfoot', export_path, encoders);

export_function(H_RLFL, 'H_RLfoot_to_FLfoot', export_path, encoders);
export_function(H_RLFL(1:3,1:3), 'R_RLfoot_to_FLfoot', export_path, encoders);
export_function(H_RLFL(1:3,end), 'p_RLfoot_to_FLfoot', export_path, encoders);
export_function(jacobian(H_RLFL(1:3,end),encoders), 'Jp_RLfoot_to_FLfoot', export_path, encoders);

end


