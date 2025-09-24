function bounds = getBounds(robot)
%% Get Bounds 
model_bounds = robot.getLimits();
bounds = struct();

% torso [x,y,z] position %
model_bounds.states.x.lb(1:3) = [-10,-0.1, 0];
model_bounds.states.x.ub(1:3) = [+10,+0.1, 3];

% torso [yaw,pitch,roll] position %
model_bounds.states.x.lb(4:6) = deg2rad([-50,-50,-50]);
model_bounds.states.x.ub(4:6) = deg2rad([+50,+50,+50]);

% robot pose: all joint zeros ==> leg straight down (calf unfeasible)
% hip joint angles -46 deg (to the right) to +46 deg (to the left)
model_bounds.states.x.lb([7 10 13 16]) = deg2rad(-20);
model_bounds.states.x.ub([7 10 13 16]) = deg2rad(+20);

% thigh joint angles -60 deg (forward) to +240 deg (backward) 
model_bounds.states.x.lb([8 11 14 17]) = deg2rad(-60);
model_bounds.states.x.ub([8 11 14 17]) = deg2rad(+90); % 240

% calf joint angles -154.5 deg (shortest) to -52.5 deg (longest)
model_bounds.states.x.lb([9 12 15 18]) = deg2rad(-120); % -154.5
model_bounds.states.x.ub([9 12 15 18]) = deg2rad(-52.5);  % -52.5

%% Multi-Domain Constraints %%

% average velocity in x (forward) direction
bounds.average_velocity.lb = [+1.4;-0];   
bounds.average_velocity.ub = [+1.6;+0];  

% last node foot velocity (implemented as constraints in each phase)
% model_bounds.swing_foot_contact_vel_FR.lb = [-0.1;-0.1;   -2;-1;-200;-1];   % + 2     
% model_bounds.swing_foot_contact_vel_FR.ub = [+0.1;+0.1;-0.01;+1;+200;+1];   % - 2   
% 
% model_bounds.swing_foot_contact_vel_FL.lb = [-0.1;-0.1;   -2;-1;-200;-1];     
% model_bounds.swing_foot_contact_vel_FL.ub = [+0.1;+0.1;-0.01;+1;+200;+1];    
% 
% model_bounds.swing_foot_contact_vel_RR.lb = [-0.1;-0.1;   -2;-1;-200;-1];     
% model_bounds.swing_foot_contact_vel_RR.ub = [+0.1;+0.1;-0.01;+1;+200;+1];    
% 
% model_bounds.swing_foot_contact_vel_RL.lb = [-0.1;-0.1;   -2;-1;-200;-1];    
% model_bounds.swing_foot_contact_vel_RL.ub = [+0.1;+0.1;-0.01;+1;+200;+1];    

%% FR Stance phase 
bounds.stance_FR = model_bounds;

% virtual constraints
bounds.stance_FR.time.t0.lb = 0;
bounds.stance_FR.time.t0.ub = 0;
bounds.stance_FR.time.t0.x0 = 0;

bounds.stance_FR.time.tf.lb = 0;
bounds.stance_FR.time.tf.ub = 1;
bounds.stance_FR.time.tf.x0 = 0.2;

bounds.stance_FR.time.duration.lb = 0.03;
bounds.stance_FR.time.duration.ub = 1;
bounds.stance_FR.time.duration.x0 = 0.2;

bounds.stance_FR.time.kp = 150;
bounds.stance_FR.time.kd = 10;

% forces
bounds.stance_FR.inputs.ConstraintWrench.fFRfoot.lb = -10000;
bounds.stance_FR.inputs.ConstraintWrench.fFRfoot.ub = 10000;
bounds.stance_FR.inputs.ConstraintWrench.fFRfoot.x0 = 100;

% contacts
bounds.stance_FR.params.pFRfoot.lb = [-10;-10;0];
bounds.stance_FR.params.pFRfoot.ub = [+10;+10;0]; 
% bounds.stance_FR.params.pFRfoot.x0 = [0.3;-0.042628;0];

bounds.stance_FR.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.stance_FR.params.atime.ub =  100*ones(6*12,1);
bounds.stance_FR.params.atime.x0 =    zeros(6*12,1);

bounds.stance_FR.params.ptime.lb = [bounds.stance_FR.time.tf.lb, bounds.stance_FR.time.t0.lb];
bounds.stance_FR.params.ptime.ub = [bounds.stance_FR.time.tf.ub, bounds.stance_FR.time.t0.ub];
bounds.stance_FR.params.ptime.x0 = [bounds.stance_FR.time.tf.x0, bounds.stance_FR.time.t0.x0];

%% FRFL Stance phase 
bounds.stance_FRFL = model_bounds;

% virtual constraints
bounds.stance_FRFL.time.t0.lb = 0;
bounds.stance_FRFL.time.t0.ub = 0;
bounds.stance_FRFL.time.t0.x0 = 0;

bounds.stance_FRFL.time.tf.lb = 0;
bounds.stance_FRFL.time.tf.ub = 1;
bounds.stance_FRFL.time.tf.x0 = 0.2;

bounds.stance_FRFL.time.duration.lb = 0.03;
bounds.stance_FRFL.time.duration.ub = 1;
bounds.stance_FRFL.time.duration.x0 = 0.2;

bounds.stance_FRFL.time.kp = 150;
bounds.stance_FRFL.time.kd = 15;

% forces
bounds.stance_FRFL.inputs.ConstraintWrench.fFRfoot.lb = -10000;
bounds.stance_FRFL.inputs.ConstraintWrench.fFRfoot.ub = 10000;
bounds.stance_FRFL.inputs.ConstraintWrench.fFRfoot.x0 = 100;

bounds.stance_FRFL.inputs.ConstraintWrench.fFLfoot.lb = -10000;
bounds.stance_FRFL.inputs.ConstraintWrench.fFLfoot.ub = 10000;
bounds.stance_FRFL.inputs.ConstraintWrench.fFLfoot.x0 = 100;

% contacts
bounds.stance_FRFL.params.pFRfoot.lb = [-10;-10;0];
bounds.stance_FRFL.params.pFRfoot.ub = [+10;+10;0];  
% bounds.stance_FRFL.params.pFRfoot.x0 = [0.3;-0.042628;0];

bounds.stance_FRFL.params.pFLfoot.lb = [-10;-10;0];
bounds.stance_FRFL.params.pFLfoot.ub = [+10;+10;0]; 
% bounds.stance_FRFL.params.pFLfoot.x0 = [0.3;0.047;0];

bounds.stance_FRFL.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.stance_FRFL.params.atime.ub =  100*ones(6*12,1);
bounds.stance_FRFL.params.atime.x0 =    zeros(6*12,1);

bounds.stance_FRFL.params.ptime.lb = [bounds.stance_FRFL.time.tf.lb, bounds.stance_FRFL.time.t0.lb];
bounds.stance_FRFL.params.ptime.ub = [bounds.stance_FRFL.time.tf.ub, bounds.stance_FRFL.time.t0.ub];
bounds.stance_FRFL.params.ptime.x0 = [bounds.stance_FRFL.time.tf.x0, bounds.stance_FRFL.time.t0.x0];

%% FL Stance phase 
bounds.stance_FL = model_bounds;

% virtual constraints
bounds.stance_FL.time.t0.lb = 0;
bounds.stance_FL.time.t0.ub = 0;
bounds.stance_FL.time.t0.x0 = 0;

bounds.stance_FL.time.tf.lb = 0;
bounds.stance_FL.time.tf.ub = 1;
bounds.stance_FL.time.tf.x0 = 0.2;

bounds.stance_FL.time.duration.lb = 0.03;
bounds.stance_FL.time.duration.ub = 1;
bounds.stance_FL.time.duration.x0 = 0.2;

bounds.stance_FL.time.kp = 150;
bounds.stance_FL.time.kd = 15;

% forces
bounds.stance_FL.inputs.ConstraintWrench.fFLfoot.lb = -10000;
bounds.stance_FL.inputs.ConstraintWrench.fFLfoot.ub = 10000;
bounds.stance_FL.inputs.ConstraintWrench.fFLfoot.x0 = 100;

% contacts
bounds.stance_FL.params.pFLfoot.lb = [-10;-10;0];
bounds.stance_FL.params.pFLfoot.ub = [+10;+10;0]; 
% bounds.stance_FL.params.pFLfoot.x0 = [0.3;0.047;0];

bounds.stance_FL.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.stance_FL.params.atime.ub =  100*ones(6*12,1);
bounds.stance_FL.params.atime.x0 =    zeros(6*12,1);

bounds.stance_FL.params.ptime.lb = [bounds.stance_FL.time.tf.lb, bounds.stance_FL.time.t0.lb];
bounds.stance_FL.params.ptime.ub = [bounds.stance_FL.time.tf.ub, bounds.stance_FL.time.t0.ub];
bounds.stance_FL.params.ptime.x0 = [bounds.stance_FL.time.tf.x0, bounds.stance_FL.time.t0.x0];

%% RR stance phase
bounds.stance_RR = model_bounds;

% virtual constraints 
bounds.stance_RR.time.t0.lb = 0; 
bounds.stance_RR.time.t0.ub = 0;  
bounds.stance_RR.time.t0.x0 = 0;  

bounds.stance_RR.time.tf.lb = 0; 
bounds.stance_RR.time.tf.ub = 1;  
bounds.stance_RR.time.tf.x0 = 0.2;  

bounds.stance_RR.time.duration.lb = 0.03; 
bounds.stance_RR.time.duration.ub = 1;  
bounds.stance_RR.time.duration.x0 = 0.2;  

bounds.stance_RR.time.kp = 150;
bounds.stance_RR.time.kd = 15;

% forces
bounds.stance_RR.inputs.ConstraintWrench.fRRfoot.lb = -10000;
bounds.stance_RR.inputs.ConstraintWrench.fRRfoot.ub = 10000;
bounds.stance_RR.inputs.ConstraintWrench.fRRfoot.x0 = 100;

% contacts
bounds.stance_RR.params.pRRfoot.lb = [-10;-10;0];
bounds.stance_RR.params.pRRfoot.ub = [+10;+10;0]; 
% bounds.stance_RR.params.pRRfoot.x0 = [-0.157538;-0.042628;0];

bounds.stance_RR.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.stance_RR.params.atime.ub =  100*ones(6*12,1);
bounds.stance_RR.params.atime.x0 =    zeros(6*12,1);

bounds.stance_RR.params.ptime.lb = [bounds.stance_RR.time.tf.lb, bounds.stance_RR.time.t0.lb];
bounds.stance_RR.params.ptime.ub = [bounds.stance_RR.time.tf.ub, bounds.stance_RR.time.t0.ub];
bounds.stance_RR.params.ptime.x0 = [bounds.stance_RR.time.tf.x0, bounds.stance_RR.time.t0.x0];

%% RRRL stance phase
bounds.stance_RRRL = model_bounds;

% virtual constraints 
bounds.stance_RRRL.time.t0.lb = 0; 
bounds.stance_RRRL.time.t0.ub = 0;  
bounds.stance_RRRL.time.t0.x0 = 0;  

bounds.stance_RRRL.time.tf.lb = 0; 
bounds.stance_RRRL.time.tf.ub = 1;  
bounds.stance_RRRL.time.tf.x0 = 0.2;  

bounds.stance_RRRL.time.duration.lb = 0.03; 
bounds.stance_RRRL.time.duration.ub = 1;  
bounds.stance_RRRL.time.duration.x0 = 0.2;  

bounds.stance_RRRL.time.kp = 150;
bounds.stance_RRRL.time.kd = 15;

% forces
bounds.stance_RRRL.inputs.ConstraintWrench.fRRfoot.lb = -10000;
bounds.stance_RRRL.inputs.ConstraintWrench.fRRfoot.ub = 10000;
bounds.stance_RRRL.inputs.ConstraintWrench.fRRfoot.x0 = 100;

bounds.stance_RRRL.inputs.ConstraintWrench.fRLfoot.lb = -10000;
bounds.stance_RRRL.inputs.ConstraintWrench.fRLfoot.ub = 10000;
bounds.stance_RRRL.inputs.ConstraintWrench.fRLfoot.x0 = 100;

% contacts
bounds.stance_RRRL.params.pRRfoot.lb = [-10;-10;0];
bounds.stance_RRRL.params.pRRfoot.ub = [+10;+10;0];
% bounds.stance_RRRL.params.pRRfoot.x0 = [-0.157538;-0.042628;0];

bounds.stance_RRRL.params.pRLfoot.lb = [-10;-10;0];
bounds.stance_RRRL.params.pRLfoot.ub = [+10;+10;0]; 
% bounds.stance_RRRL.params.pRLfoot.x0 = [-0.157538;0.047;0];

bounds.stance_RRRL.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.stance_RRRL.params.atime.ub =  100*ones(6*12,1);
bounds.stance_RRRL.params.atime.x0 =    zeros(6*12,1);

bounds.stance_RRRL.params.ptime.lb = [bounds.stance_RRRL.time.tf.lb, bounds.stance_RRRL.time.t0.lb];
bounds.stance_RRRL.params.ptime.ub = [bounds.stance_RRRL.time.tf.ub, bounds.stance_RRRL.time.t0.ub];
bounds.stance_RRRL.params.ptime.x0 = [bounds.stance_RRRL.time.tf.x0, bounds.stance_RRRL.time.t0.x0];

%% RL stance phase
bounds.stance_RL = model_bounds;

% virtual constraints 
bounds.stance_RL.time.t0.lb = 0; 
bounds.stance_RL.time.t0.ub = 0;  
bounds.stance_RL.time.t0.x0 = 0;  

bounds.stance_RL.time.tf.lb = 0; 
bounds.stance_RL.time.tf.ub = 1;  
bounds.stance_RL.time.tf.x0 = 0.2;  

bounds.stance_RL.time.duration.lb = 0.03; 
bounds.stance_RL.time.duration.ub = 1;  
bounds.stance_RL.time.duration.x0 = 0.2;  

bounds.stance_RL.time.kp = 150;
bounds.stance_RL.time.kd = 15;

% forces
bounds.stance_RL.inputs.ConstraintWrench.fRLfoot.lb = -10000;
bounds.stance_RL.inputs.ConstraintWrench.fRLfoot.ub = 10000;
bounds.stance_RL.inputs.ConstraintWrench.fRLfoot.x0 = 100;

% contacts
bounds.stance_RL.params.pRLfoot.lb = [-10;-10;0];
bounds.stance_RL.params.pRLfoot.ub = [+10;+10;0]; 
% bounds.stance_RL.params.pRLfoot.x0 = [-0.157538;-0.042628;0];

bounds.stance_RL.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.stance_RL.params.atime.ub =  100*ones(6*12,1);
bounds.stance_RL.params.atime.x0 =    zeros(6*12,1);

bounds.stance_RL.params.ptime.lb = [bounds.stance_RL.time.tf.lb, bounds.stance_RL.time.t0.lb];
bounds.stance_RL.params.ptime.ub = [bounds.stance_RL.time.tf.ub, bounds.stance_RL.time.t0.ub];
bounds.stance_RL.params.ptime.x0 = [bounds.stance_RL.time.tf.x0, bounds.stance_RL.time.t0.x0];

%% Flight1 phase
bounds.flight1 = model_bounds;

% virtual constraints 
bounds.flight1.time.t0.lb = 0;
bounds.flight1.time.t0.ub = 0;
bounds.flight1.time.t0.x0 = 0;

bounds.flight1.time.tf.lb = 0;
bounds.flight1.time.tf.ub = 1;
bounds.flight1.time.tf.x0 = 0.2;

bounds.flight1.time.duration.lb = 0.03;
bounds.flight1.time.duration.ub = 1;
bounds.flight1.time.duration.x0 = 0.2;

bounds.flight1.time.kp = 150;
bounds.flight1.time.kd = 15;

bounds.flight1.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.flight1.params.atime.ub =  100*ones(6*12,1);
bounds.flight1.params.atime.x0 =    zeros(6*12,1);

bounds.flight1.params.ptime.lb = [bounds.flight1.time.tf.lb, bounds.flight1.time.t0.lb];
bounds.flight1.params.ptime.ub = [bounds.flight1.time.tf.ub, bounds.flight1.time.t0.ub];
bounds.flight1.params.ptime.x0 = [bounds.flight1.time.tf.x0, bounds.flight1.time.t0.x0];

%% Flight2 phase
bounds.flight2 = model_bounds;

% virtual constraints 
bounds.flight2.time.t0.lb = 0;
bounds.flight2.time.t0.ub = 0;
bounds.flight2.time.t0.x0 = 0;

bounds.flight2.time.tf.lb = 0;
bounds.flight2.time.tf.ub = 1;
bounds.flight2.time.tf.x0 = 0.2;

bounds.flight2.time.duration.lb = 0.03;
bounds.flight2.time.duration.ub = 1;
bounds.flight2.time.duration.x0 = 0.2;

bounds.flight2.time.kp = 150;
bounds.flight2.time.kd = 15;

bounds.flight2.params.atime.lb = -100*ones(6*12,1); % 5th order polynomial * 12 joints
bounds.flight2.params.atime.ub =  100*ones(6*12,1);
bounds.flight2.params.atime.x0 =    zeros(6*12,1);

bounds.flight2.params.ptime.lb = [bounds.flight2.time.tf.lb, bounds.flight2.time.t0.lb];
bounds.flight2.params.ptime.ub = [bounds.flight2.time.tf.ub, bounds.flight2.time.t0.ub];
bounds.flight2.params.ptime.x0 = [bounds.flight2.time.tf.x0, bounds.flight2.time.t0.x0];

%% discrete events

bounds.FR_td = model_bounds;
bounds.FR_td.inputs.ConstraintWrench.fFRfoot.lb = -10000;
bounds.FR_td.inputs.ConstraintWrench.fFRfoot.ub = 10000;

bounds.FL_td = model_bounds;
bounds.FL_td.inputs.ConstraintWrench.fFLfoot.lb = -10000;
bounds.FL_td.inputs.ConstraintWrench.fFLfoot.ub = 10000;

bounds.RR_td = model_bounds;
bounds.RR_td.inputs.ConstraintWrench.fRRfoot.lb = -10000;
bounds.RR_td.inputs.ConstraintWrench.fRRfoot.ub = 10000;

bounds.RL_td = model_bounds;
bounds.RL_td.inputs.ConstraintWrench.fRLfoot.lb = -10000;
bounds.RL_td.inputs.ConstraintWrench.fRLfoot.ub = 10000;

bounds.FR_lo = model_bounds;
bounds.FL_lo = model_bounds;
bounds.RR_lo = model_bounds;
bounds.RL_lo = model_bounds;

end

