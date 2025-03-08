function RLConstraint(nlp, bounds, varargin)
%% Add Virtual Constraints
    nlp.Plant.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
%% Tau Boundary [0,1]
    addNodeConstraint(nlp,AllConstraints.Tau0(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName],'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp,AllConstraints.TauF(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
    
%% Unilateral Constraints 
%             FRFK = nlp.Plant.ContactPoints.FRfoot.computeForwardKinematics;  %  Front-Right Foot Forward Kinematics 
%              Fz1 = FRFK(3,4);
%           Fz1Fun = SymFunction('FRFootHeight', Fz1, {nlp.Plant.States.x}); 
%           addNodeConstraint(nlp,Fz1Fun,{'x'},5:15,0.03,inf,'Nonlinear');
% 
%             FLFK = nlp.Plant.ContactPoints.FLfoot.computeForwardKinematics;  %  Front-Left Foot Forward Kinematics 
%              Fz2 = FLFK(3,4); 
%           Fz2Fun = SymFunction('FLFootHeight', Fz2, {nlp.Plant.States.x}); 
%           addNodeConstraint(nlp,Fz2Fun,{'x'},5:15,0.03,inf,'Nonlinear');
  
%% Costs 
% Rear Leg Torque 
%              w = 1;
%              u = nlp.Plant.Inputs.Control.u([8,9,11,12]);
%           cost = sum((w.*u).^2);
% TorqueRearFun = SymFunction('TorqueRear', cost, {nlp.Plant.Inputs.Control.u});
%   
%           addRunningCost(nlp, TorqueRearFun, 'u');
%      
% Front Leg Velocity
%                   w = [1,1,1,1];
%                  dq = nlp.Plant.States.dx([8,9,11,12]);
%                cost = (w(1)*dq(1)).^2 + (w(2)*dq(2)).^2  + (w(3)*dq(3)).^2  + (w(4)*dq(4)).^2;
% FrontLegVelocityFun = SymFunction('FrontLegVelocity', cost, {nlp.Plant.States.dx});
% 
%            addRunningCost(nlp, FrontLegVelocityFun, 'dx');
 
%%

      addRunningCost(nlp,AllCosts.Torque(nlp),'u');
      
end
