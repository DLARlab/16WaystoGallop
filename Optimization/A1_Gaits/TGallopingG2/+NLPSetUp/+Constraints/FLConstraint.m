function FLConstraint(nlp, bounds, varargin)
%% Add Virtual Constraints
    nlp.Plant.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
%% Tau Boundary [0,1]
    addNodeConstraint(nlp,AllConstraints.Tau0(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName],'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp,AllConstraints.TauF(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    
%% Unilateral Constraints
%             RRFK = nlp.Plant.ContactPoints.RRfoot.computeForwardKinematics;  %  Rear-Right Foot Forward Kinematics 
%              Fz1 = RRFK(3,4);
%           Fz1Fun = SymFunction('RRFootHeight',Fz1,{nlp.Plant.States.x}); 
%           addNodeConstraint(nlp, Fz1Fun,{'x'},5:15,0.03,inf,'Nonlinear');
% 
%             RLFK = nlp.Plant.ContactPoints.RLfoot.computeForwardKinematics;  %  Rear-Left Foot Forward Kinematics 
%              Fz2 = RLFK(3,4); 
%           Fz2Fun = SymFunction('RLFootHeight',Fz2,{nlp.Plant.States.x}); 
%           addNodeConstraint(nlp,Fz2Fun,{'x'},5:15,0.03,inf,'Nonlinear');
    
%%  Cost 
% % Front Leg Torque 
%              w = 1;
%              u = nlp.Plant.Inputs.Control.u([2,3,5,6]);
%           cost = sum((w.*u).^2);
% TorqueFrontFun = SymFunction('TorqueFront', cost, {nlp.Plant.Inputs.Control.u});
%   
%           addRunningCost(nlp, TorqueFrontFun, 'u');
%      
% % Rear Leg Velocity
%                   w = [1,1,1,1];
%                  dq = nlp.Plant.States.dx([14,15,17,18]);
%                cost = (w(1)*dq(1)).^2 + (w(2)*dq(2)).^2  + (w(3)*dq(3)).^2  + (w(4)*dq(4)).^2;
% RearLegVelocityFun = SymFunction('RearLegVelocity', cost, {nlp.Plant.States.dx});
% 
%            addRunningCost(nlp, RearLegVelocityFun, 'dx');

%%
 
      addRunningCost(nlp,AllCosts.Torque(nlp),'u');
      
end
