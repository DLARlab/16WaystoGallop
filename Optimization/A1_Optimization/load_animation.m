function [conGUI] = load_animation(robot, gait, terrain, varargin)

      
    ip = inputParser;
    ip.addParameter('UseExported',true,@(x) isequal(x,true) || isequal(x,false));
    ip.addParameter('ExportPath','',@(x)ischar(x));
    ip.addParameter('SkipExporting',false,@(x) isequal(x,true) || isequal(x,false));
    
    ip.parse(varargin{:});
    
    opts = ip.Results;
    if isempty(opts.ExportPath)
        root_path = get_root_path();
        export_path = fullfile(root_path,'gen','anim');
        opts.ExportPath = export_path;
    else
        export_path = opts.ExportPath;
    end
    if ~exist(export_path,'dir')
        mkdir(export_path);
    end
    addpath(export_path);
    
    cont_domain_idx = find(arrayfun(@(x)~isempty(x.tspan),gait));
    t = [];
    q = [];
    
    for j=cont_domain_idx.'
        t = [t,gait(j).tspan];         %#ok<*AGROW>
        q = [q,gait(j).states.x];
    end
    

    f = figure(1000); clf;
    
    % Add the skeleton for A1 robot
    a1_disp = frost.Animator.Display(f, robot, opts);
    
    
    % Add patches for A1 robot
    % Add Trunk
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    trunk = robot.Links(getLinkIndices(robot,'trunk'));
    A1_trunk_anim_frame = CoordinateFrame(...
         'Name','FRfootAnim',...
         'Reference',trunk.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'trunk.stl'; 
    funcname = 'H_BaseRotX';
    A1_trunk = frost.Animator.PatchSTL(a1_disp.axs, robot, A1_trunk_anim_frame, filename, funcname, 'Trunk', opts);
    a1_disp.addItem(A1_trunk);    

    % Add patches for FR leg
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Add FR Hip
    FR_hip_Link = robot.Links(getLinkIndices(robot,'FR_hip'));
    FR_hip_anim_frame = CoordinateFrame(...
         'Name','FRhipAnim',...
         'Reference',FR_hip_Link.Reference,...
         'Offset', [0,0,0],...
         'R',[0,0,0]);
    filename = 'hip.stl'; 
    funcname = 'H_FR_hip_joint'; 
    FR_hip = frost.Animator.PatchSTL(a1_disp.axs, robot, FR_hip_anim_frame, filename, funcname, 'FRhip', opts);
    a1_disp.addItem(FR_hip);    
    
    % Add FR thigh
    FR_thigh_Link = robot.Links(getLinkIndices(robot,'FR_thigh'));
    FR_thigh_anim_frame = CoordinateFrame(...
         'Name','FRhipAnim',...
         'Reference',FR_thigh_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'thigh_mirror.stl'; 
    funcname = 'H_FR_thigh_joint'; 
    FR_thigh = frost.Animator.PatchSTL(a1_disp.axs, robot, FR_thigh_anim_frame, filename, funcname, 'FRthigh', opts);
    a1_disp.addItem(FR_thigh);   
    
    % Add FR calf
    FR_calf_Link = robot.Links(getLinkIndices(robot,'FR_calf'));
    FR_calf_anim_frame = CoordinateFrame(...
         'Name','FRhipAnim',...
         'Reference',FR_calf_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'calf.stl'; 
    funcname = 'H_FR_calf_joint'; 
    FR_calf = frost.Animator.PatchSTL(a1_disp.axs, robot, FR_calf_anim_frame, filename, funcname, 'FRcalf', opts);
    a1_disp.addItem(FR_calf);   
    
    
    % Add patches for FL leg
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Add FL Hip
    FL_hip_Link = robot.Links(getLinkIndices(robot,'FL_hip'));
    FL_hip_anim_frame = CoordinateFrame(...
         'Name','FLhipAnim',...
         'Reference',FL_hip_Link.Reference,...
         'Offset', [0,0,0],...
         'R',[0,0,0]);
    filename = 'hip.stl'; 
    funcname = 'H_FL_hip_joint'; 
    FL_hip = frost.Animator.PatchSTL(a1_disp.axs, robot, FL_hip_anim_frame, filename, funcname, 'FLhip', opts);
    a1_disp.addItem(FL_hip);    
    
    % Add FL thigh
    FL_thigh_Link = robot.Links(getLinkIndices(robot,'FL_thigh'));
    FL_thigh_anim_frame = CoordinateFrame(...
         'Name','FLhipAnim',...
         'Reference',FL_thigh_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'thigh.stl'; 
    funcname = 'H_FL_thigh_joint'; 
    FL_thigh = frost.Animator.PatchSTL(a1_disp.axs, robot, FL_thigh_anim_frame, filename, funcname, 'FLthigh', opts);
    a1_disp.addItem(FL_thigh);   
    
    % Add FL calf
    FL_calf_Link = robot.Links(getLinkIndices(robot,'FL_calf'));
    FL_calf_anim_frame = CoordinateFrame(...
         'Name','FLhipAnim',...
         'Reference',FL_calf_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'calf.stl'; 
    funcname = 'H_FL_calf_joint'; 
    FL_calf = frost.Animator.PatchSTL(a1_disp.axs, robot, FL_calf_anim_frame, filename, funcname, 'FLcalf', opts);
    a1_disp.addItem(FL_calf);     
    

    % Add patches for RR leg
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Add RR Hip
    RR_hip_Link = robot.Links(getLinkIndices(robot,'RR_hip'));
    RR_hip_anim_frame = CoordinateFrame(...
         'Name','RRhipAnim',...
         'Reference',RR_hip_Link.Reference,...
         'Offset', [0,0,0],...
         'R',[0,pi,0]);
    filename = 'hip.stl'; 
    funcname = 'H_RR_hip_joint'; 
    RR_hip = frost.Animator.PatchSTL(a1_disp.axs, robot, RR_hip_anim_frame, filename, funcname, 'RRhip', opts);
    a1_disp.addItem(RR_hip);    
    
    % Add RR thigh
    RR_thigh_Link = robot.Links(getLinkIndices(robot,'RR_thigh'));
    RR_thigh_anim_frame = CoordinateFrame(...
         'Name','RRhipAnim',...
         'Reference',RR_thigh_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'thigh_mirror.stl'; 
    funcname = 'H_RR_thigh_joint'; 
    RR_thigh = frost.Animator.PatchSTL(a1_disp.axs, robot, RR_thigh_anim_frame, filename, funcname, 'RRthigh', opts);
    a1_disp.addItem(RR_thigh);   
    
    % Add RR calf
    RR_calf_Link = robot.Links(getLinkIndices(robot,'RR_calf'));
    RR_calf_anim_frame = CoordinateFrame(...
         'Name','RRhipAnim',...
         'Reference',RR_calf_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'calf.stl'; 
    funcname = 'H_RR_calf_joint'; 
    RR_calf = frost.Animator.PatchSTL(a1_disp.axs, robot, RR_calf_anim_frame, filename, funcname, 'RRcalf', opts);
    a1_disp.addItem(RR_calf);    
  
   
    % Add patches for RL leg
    % +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    % Add RL Hip
    RL_hip_Link = robot.Links(getLinkIndices(robot,'RL_hip'));
    RL_hip_anim_frame = CoordinateFrame(...
         'Name','RLhipAnim',...
         'Reference',RL_hip_Link.Reference,...
         'Offset', [0,0,0],...
         'R',[0,pi,0]);
    filename = 'hip.stl'; 
    funcname = 'H_RL_hip_joint'; 
    RL_hip = frost.Animator.PatchSTL(a1_disp.axs, robot, RL_hip_anim_frame, filename, funcname, 'RLhip', opts);
    a1_disp.addItem(RL_hip);    
    
    % Add RL thigh
    RL_thigh_Link = robot.Links(getLinkIndices(robot,'RL_thigh'));
    RL_thigh_anim_frame = CoordinateFrame(...
         'Name','RLhipAnim',...
         'Reference',RL_thigh_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'thigh.stl'; 
    funcname = 'H_RL_thigh_joint'; 
    RL_thigh = frost.Animator.PatchSTL(a1_disp.axs, robot, RL_thigh_anim_frame, filename, funcname, 'RLthigh', opts);
    a1_disp.addItem(RL_thigh);   
    
    % Add RL calf
    RL_calf_Link = robot.Links(getLinkIndices(robot,'RL_calf'));
    RL_calf_anim_frame = CoordinateFrame(...
         'Name','RLhipAnim',...
         'Reference',RL_calf_Link.Reference,...
         'Offset',[0,0,0],...
         'R',[0,0,0]);
    filename = 'calf.stl'; 
    funcname = 'H_RL_calf_joint'; 
    RL_calf = frost.Animator.PatchSTL(a1_disp.axs, robot, RL_calf_anim_frame, filename, funcname, 'RLcalf', opts);
    a1_disp.addItem(RL_calf);    
    
    
    
    % Display
    if isempty(terrain)
        [terrain.Tx, terrain.Ty] = meshgrid(-5:1:5, -5:1:5);
        terrain.Tz = 0.*terrain.Tx;
    end
    anim = frost.Animator.AbstractAnimator(a1_disp, t, q, terrain);
    anim.isLooping = true;
    anim.speed = 1;
    anim.pov = frost.Animator.AnimatorPointOfView.Free;
    anim.Animate(true);
    conGUI = frost.Animator.AnimatorControls();
    conGUI.anim = anim;
    
end