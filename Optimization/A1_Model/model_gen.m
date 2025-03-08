% Reset 
clear; matlabrc; restoredefaultpath; clc;
cur = pwd;
EXPORT_PATH = fullfile(cur, 'gen');
if ~exist(EXPORT_PATH,'dir')
    mkdir(EXPORT_PATH);
    addpath(EXPORT_PATH);
end

% Add FROST
FROST_PATH = '../frost-dev';
addpath(FROST_PATH)
frost_addpath;

% Add model paths
addpath(genpath('urdf'));

%% Load model
a1 = A1(fullfile('urdf','a1.urdf')); 


%% Export Kinematics
if ~exist(fullfile(EXPORT_PATH,'kin'),'dir')
    mkdir(fullfile(EXPORT_PATH,'kin'));
    mkdir(fullfile(EXPORT_PATH,'kin','slrt'));
    mkdir(fullfile(EXPORT_PATH,'kin','eigen'));
end
a1.ExportKinematics(@Export.export_slrt, fullfile(EXPORT_PATH,'kin','slrt'));
a1.ExportKinematics(@Export.export_eigen, fullfile(EXPORT_PATH,'kin','eigen'));    
disp('Done')
%% Export Dynamics
if ~exist(fullfile(EXPORT_PATH,'dyn'),'dir')
    mkdir(fullfile(EXPORT_PATH,'dyn'));
    mkdir(fullfile(EXPORT_PATH,'dyn','slrt'));
    mkdir(fullfile(EXPORT_PATH,'dyn','eigen'));
end
a1.configureDynamics('DelayCoriolisSet',false);
a1.ExportDynamics(@Export.export_slrt, fullfile(EXPORT_PATH,'dyn','slrt'));    
a1.ExportDynamics(@Export.export_eigen, fullfile(EXPORT_PATH,'dyn','eigen'));    
% a1.compile([EXPORT_PATH,'dyn/']);
disp('Done')
%% Compute Manipulator Jacobians
% a1.ExportKinematics_IMU(@Export.export_slrt, fullfile(EXPORT_PATH,'kin','slrt'));
% a1.ExportKinematics_IMU(@Export.export_eigen, fullfile(EXPORT_PATH,'kin','eigen'));
%     
% a1.ExportJacobians_IMU(@Export.export_slrt, fullfile(EXPORT_PATH,'kin','slrt'));
% a1.ExportJacobians_IMU(@Export.export_eigen, fullfile(EXPORT_PATH,'kin','eigen'));

%% Export Energy
% a1.ExportEnergy(@Export.export_slrt, [EXPORT_PATH,'dyn/']);

%% Compute CoM position/velocity
% a1.ExportCoM(@Export.export_slrt, [EXPORT_PATH,'kin/']);

%% Compute/export dynamics
% a1.ExportHolonomicConstraints(@Export.export_slrt, [EXPORT_PATH,'dyn/']);
% a1.ExportMomentum(@Export.export_slrt, [EXPORT_PATH,'dyn/']);




