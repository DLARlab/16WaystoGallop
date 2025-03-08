function FRRLConstraint(nlp, bounds, varargin)
%% Add Virtual Constraints
    nlp.Plant.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
%% Tau Boundary [0,1]
    addNodeConstraint(nlp,AllConstraints.Tau0(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName],'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp,AllConstraints.TauF(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
            
%% Unilateral Constraints   
            FLFK = nlp.Plant.ContactPoints.FLfoot.computeCartesianPosition;  %  Front-Left Foot Forward Kinematics 
             Fz2 = FLFK(3);
          Fz2Fun = SymFunction('FLFootHeight',Fz2,{nlp.Plant.States.x}); 
          addNodeConstraint(nlp, Fz2Fun,{'x'},8:10,0.02,inf,'Nonlinear');
          
            RRFK = nlp.Plant.ContactPoints.RRfoot.computeCartesianPosition;  %  Rear-Right Foot Forward Kinematics 
             Fz3 = RRFK(3);
          Fz3Fun = SymFunction('RRFootHeight',Fz3,{nlp.Plant.States.x}); 
          addNodeConstraint(nlp, Fz3Fun,{'x'},8:10,0.02,inf,'Nonlinear');

%% Costs         
%  Leg Angle velocity
%       addRunningCost(nlp,AllCosts.Torque(nlp),'u');
      
end
