%% 1- Setup
clear; close all; clc;
restoredefaultpath; matlabrc;
% specify the path to the FROST
frost_path  = '../../';
addpath(frost_path);
frost_addpath; % initialize FROST
export_path = 'gen/opt'; % path to export compiled C++ and MEX files
    if ~exist(export_path,'dir')
        mkdir(export_path);
    end
    addpath(export_path);
   
%% 2- Settings %%
LOAD = false;  % load symbolic expressions instead of direct evaluation to save time, must save the symbolic expresssion first 
COMPILE = true; % compile MEX binaries
SAVE = true;    % save symbolic expressions for load directly
OMIT_CORIOLIS = false; % drop veloci terms

%% 3- Load Hybrid System %%
robot = A1(fullfile(root, 'A1_Model','urdf','a1Original.urdf'));  
if LOAD
    robot.loadDynamics(LOAD_PATH, OMIT_CORIOLIS,{},'OmitCoriolisSet',OMIT_CORIOLIS); 
    [sys, domains, guards] = load_behavior(robot, LOAD_PATH,'type','RGallopingG2');
else
    robot.configureDynamics('DelayCoriolisSet',OMIT_CORIOLIS,'OmitCoriolisSet',OMIT_CORIOLIS); 
    [sys, domains, guards] = load_behavior(robot,[], 'type','RGallopingG2');
end
%plot(sys.Gamma)
                                        
%% 4- Create Optimization Problem %
% Specifying the Number of collocation points in the stance phase and flight phase %
  num_grid.stance_FR = 8; 
num_grid.stance_FRFL = 8;
  num_grid.stance_FL = 8;
    num_grid.flight1 = 5;
  num_grid.stance_RL = 8; 
num_grid.stance_RRRL = 8;
  num_grid.stance_RR = 8;
    num_grid.flight2 = 5;
nlp = HybridTrajectoryOptimization('A1_RG2',sys,num_grid,[],'EqualityConstraintBoundary',1e-3);

%% 5- Add Domain (Phase) Specific Constraints %%
nlp.Phase(1).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.FRConstraint;     
nlp.Phase(2).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.EventConstraint;     
nlp.Phase(3).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.FRFLConstraint;     % flight1
nlp.Phase(4).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.EventConstraint;  
nlp.Phase(5).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.FLConstraint;      
nlp.Phase(6).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.EventConstraint;     
nlp.Phase(7).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.Flight1Constraint;    
nlp.Phase(8).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.EventConstraint;      
nlp.Phase(9).Plant.UserNlpConstraint  = @NLPSetUp.Constraints.RLConstraint;     
nlp.Phase(10).Plant.UserNlpConstraint = @NLPSetUp.Constraints.EventConstraint;     
nlp.Phase(11).Plant.UserNlpConstraint = @NLPSetUp.Constraints.RRRLConstraint;   
nlp.Phase(12).Plant.UserNlpConstraint = @NLPSetUp.Constraints.EventConstraint;  
nlp.Phase(13).Plant.UserNlpConstraint = @NLPSetUp.Constraints.RRConstraint;      
nlp.Phase(14).Plant.UserNlpConstraint = @NLPSetUp.Constraints.EventConstraint;     
nlp.Phase(15).Plant.UserNlpConstraint = @NLPSetUp.Constraints.Flight2Constraint;   % flight2
nlp.Phase(16).Plant.UserNlpConstraint = @NLPSetUp.Constraints.EventConstraint;      
nlp.update;

%% 6- Configure Bounds and Update %%
bounds = NLPSetUp.Bounds.getBounds(robot); 
if LOAD
    nlp.configure(bounds, LOAD_PATH);
else
    nlp.configure(bounds); %#ok<UNRCH>
end

%% 7- Add Multi-Domain Constraints %%
nlp = NLPSetUp.Constraints.MultidomainConstraints(nlp,bounds);
nlp.update;

%% 8- Compile Single Constraint or Cost Function %%
% Compile Constraint:
          compileConstraint(nlp, 15, {'SwingFootContactVelocity_FR'}, EXPORT_PATH, 'dynamics_equation');

% Compile Cost Function:  
%               compileObjective(nlp, 15, {'COTdisHS'}, EXPORT_PATH,'dynamics_equation'); 
 
% Remove Single Constraint:
%          removeConstraint(nlp.Phase(1),'dynamics_equation');

%% 9- Compile All Constarints and Cost Functions %%
if COMPILE
     compileConstraint(nlp,[],[],EXPORT_PATH, []);
     compileObjective(nlp,[],[],EXPORT_PATH, []);
end

if SAVE
    if ~exist(LOAD_PATH,'dir') %#ok<UNRCH>
        mkdir(LOAD_PATH);
    end
    sys.saveExpression(LOAD_PATH);
end

%% 10- NLP Settings
average_velocity = 6;
nlp.Phase(15).updateConstrProp('average_velocity_entire_stride', 'last','lb', [average_velocity-0.01;0], 'ub', [average_velocity+0.01;0]); 

%% 11a- Solving the NLP Problem with default IC %%
solver = IpoptApplication(nlp);
[sol, info] = optimize(solver);
gait = NLPSetUp.Utilities.parse(nlp,sol);
% ----cost---- %
% Phase_Cost = checkCosts(nlp, sol);

%% 11b- Solving the NLP Problem with new IC %%
x0  = sol;
solver = IpoptApplication(nlp);
[sol, info] = optimize(solver,x0);
gait = NLPSetUp.Utilities.parse(nlp, sol);
% ----save---- %
% save([pwd,'\Velocity_',num2str(average_velocity),'.mat'],'nlp','gait','info','sol','bounds')
% save([pwd,'\Velocity_',num2str(average_velocity),'sol',Specs,'.mat'],'sol')

%% 13- Animation %%
close all
skip_export = true; % set it to true after exporting the functions once.
load_animation(robot, gait, [], 'ExportPath', ANIM_PATH, 'SkipExporting', skip_export);

%% 14- Variables, Constraints, and Cost log %%
checkVariables(nlp, sol, 1e-4,'VariablesLog.txt');
checkConstraints(nlp, sol,1e-0, 'ConstraintsLog.txt');
checkCosts(nlp, sol,'CostLog.txt');

%% 15- Ploting The Torque and Trajectory of Each Joint%%
PlotA1(gait,'JointsTorques')
PlotA1(gait,'JointsPositions')
PlotA1(gait,'BodyPositions')
   