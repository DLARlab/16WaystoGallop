function FRRRConstraint(nlp, bounds, varargin)
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
          
            RLFK = nlp.Plant.ContactPoints.RLfoot.computeCartesianPosition;  %  Rear-Left Foot Forward Kinematics 
             Fz4 = RLFK(3); 
          Fz4Fun = SymFunction('RLFootHeight',Fz4,{nlp.Plant.States.x}); 
          addNodeConstraint(nlp,Fz4Fun,{'x'},8:10,0.02,inf,'Nonlinear');
    
%% Costs         
%       addRunningCost(nlp,AllCosts.Torque(nlp),'u');
      
end
