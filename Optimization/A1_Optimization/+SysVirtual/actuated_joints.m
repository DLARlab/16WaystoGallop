function [ y2 ] = actuated_joints( robot, load_path )
% Virtual contraint consisting of all actuated joints

% phase variable: time
t = SymVariable('t');
p = SymVariable('p',[2,1]);
tau = (t-p(2))/(p(1)-p(2));

% The names of the states have to match the ones in urdf

% relative degree two outputs:
y_FR_hip    = robot.States.x('FR_hip_joint');
y_FR_thigh  = robot.States.x('FR_thigh_joint');
y_FR_calf   = robot.States.x('FR_calf_joint'); 

y_FL_hip    = robot.States.x('FL_hip_joint');
y_FL_thigh  = robot.States.x('FL_thigh_joint');
y_FL_calf   = robot.States.x('FL_calf_joint');

y_RR_hip    = robot.States.x('RR_hip_joint');
y_RR_thigh  = robot.States.x('RR_thigh_joint');
y_RR_calf   = robot.States.x('RR_calf_joint');

y_RL_hip    = robot.States.x('RL_hip_joint'); % left hind hip abduction
y_RL_thigh  = robot.States.x('RL_thigh_joint'); % left hind hip pitch (flexion/extension)
y_RL_calf   = robot.States.x('RL_calf_joint'); % left hind knee 


%ya_new = robot.States.x('BasePosX') - 10.0 * exp(3*t);

ya_2 = [y_FR_hip;
        y_FR_thigh;
        y_FR_calf;
        y_FL_hip;
        y_FL_thigh;
        y_FL_calf;        
        y_RR_hip;
        y_RR_thigh;
        y_RR_calf;
        y_RL_hip;
        y_RL_thigh;
        y_RL_calf;
        ];

y2_label = {'FR_hip', 'FR_thigh', 'FR_calf',...
            'FL_hip', 'FL_thigh', 'FL_calf',...
            'RR_hip', 'RR_thigh', 'RR_calf',...
            'RL_hip', 'RL_thigh', 'RL_calf'};

y2 = VirtualConstraint(robot,ya_2,'time','DesiredType','Bezier','PolyDegree',5,...
    'RelativeDegree',2,'OutputLabel',{y2_label},'PhaseType','TimeBased',...
    'PhaseVariable',tau,'PhaseParams',p,'Holonomic',true, 'LoadPath', load_path);

end

