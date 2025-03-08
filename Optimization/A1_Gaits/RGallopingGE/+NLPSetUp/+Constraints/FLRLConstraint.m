function FLRLConstraint(nlp, bounds, varargin)
 %% Add Virtual Constraints
    nlp.Plant.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
%% Tau Boundary [0,1]
    addNodeConstraint(nlp,AllConstraints.Tau0(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName],'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp,AllConstraints.TauF(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    
%% Final node foot velocity
%     addNodeConstraint(nlp, AllConstraints.SwingFootContactVelocity(nlp,'FR'), ...
%         {'x','dx'}, 'last', bounds.swing_foot_contact_vel_FR.lb, bounds.swing_foot_contact_vel_FR.ub, 'Nonlinear');
% 
%     addNodeConstraint(nlp, AllConstraints.SwingFootContactVelocity(nlp,'FL'), ...
%         {'x','dx'}, 'last', bounds.swing_foot_contact_vel_FL.lb, bounds.swing_foot_contact_vel_FL.ub, 'Nonlinear');
       
%% Unilateral Constraints
%             FRFK = nlp.Plant.ContactPoints.FRfoot.computeForwardKinematics;  %  Front-Right Foot Forward Kinematics 
%              Fz1 = FRFK(3,4);
%           Fz1Fun = SymFunction('FRFootHeight',Fz1,{nlp.Plant.States.x}); 
%           addNodeConstraint(nlp,Fz1Fun,{'x'},4:8,0.04,inf,'Nonlinear');
% 
%             FLFK = nlp.Plant.ContactPoints.FLfoot.computeForwardKinematics;  %  Front-Left Foot Forward Kinematics 
%              Fz2 = FLFK(3,4); 
%           Fz2Fun = SymFunction('FLFootHeight',Fz2,{nlp.Plant.States.x}); 
%           addNodeConstraint(nlp,Fz2Fun,{'x'},4:8,0.04,inf,'Nonlinear');
%   
%             RRFK = nlp.Plant.ContactPoints.RRfoot.computeForwardKinematics;  %  Rear-Right Foot Forward Kinematics 
%              Fz1 = RRFK(3,4);
%           Fz1Fun = SymFunction('RRFootHeight',Fz1,{nlp.Plant.States.x}); 
%           addNodeConstraint(nlp,Fz1Fun,{'x'},4:8,0.04,inf,'Nonlinear');
% 
%             RLFK = nlp.Plant.ContactPoints.RLfoot.computeForwardKinematics;  %  Rear-Left Foot Forward Kinematics 
%              Fz2 = RLFK(3,4); 
%           Fz2Fun = SymFunction('RLFootHeight',Fz2,{nlp.Plant.States.x}); 
%           addNodeConstraint(nlp,Fz2Fun,{'x'},4:8,0.04,inf,'Nonlinear');
          
%% Touchdown Angle Front Legs %%      
%                FRFFK = nlp.Plant.ContactPoints.FRfoot.computeForwardKinematics;    %  Front Right Foot Forward Kinematics 
%                FLFFK = nlp.Plant.ContactPoints.FLfoot.computeForwardKinematics;    %  Front Left Foot Forward Kinematics   
%                FRTFK = nlp.Plant.Joints(8).computeForwardKinematics;               %  Front Right Thigh Forward Kinematics
%                FLTFK = nlp.Plant.Joints(11).computeForwardKinematics;              %  Front Left Thigh Forward Kinematics 
%                 FRFx = FRFFK(1,4);
%                 FRTx = FRTFK(1,4); 
%                 FLFx = FLFFK(1,4);
%                 FLTx = FLTFK(1,4);
%               
%       TouchdownAngle = [FRFx - FRTx; FLFx - FLTx];  
%    TouchdownAngleFun = SymFunction('TouchdownAngleF', TouchdownAngle, {nlp.Plant.States.x});
%  
%      addNodeConstraint(nlp, TouchdownAngleFun,{'x'}, 'last', [0;0],[inf;inf], 'Nonlinear');     

%% Costs         
%  Leg Angle velocity
%       addRunningCost(nlp,AllCosts.AllLegsVelocity(nlp),'dx');
      addRunningCost(nlp,AllCosts.Torque(nlp),'u');
      
end
