function Flight1Constraint(nlp, bounds, varargin)
 %% Add Virtual Constraints
    nlp.Plant.VirtualConstraints.time.imposeNLPConstraint(nlp, [bounds.time.kp, bounds.time.kd], [1,1]);
    
%% Tau Boundary [0,1]
    addNodeConstraint(nlp,AllConstraints.Tau0(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName],'first', 0, 0, 'Nonlinear');
    addNodeConstraint(nlp,AllConstraints.TauF(nlp),[{'T'},nlp.Plant.VirtualConstraints.time.PhaseParamName], 'last', 0, 0, 'Nonlinear');
       
%% Unilateral Constraints
            FRFK = nlp.Plant.ContactPoints.FRfoot.computeCartesianPosition;  %  Front-Right Foot Forward Kinematics 
             Fz1 = FRFK(3);
          Fz1Fun = SymFunction('FRFootHeight',Fz1,{nlp.Plant.States.x}); 
          addNodeConstraint(nlp,Fz1Fun,{'x'},5:7,0.04,inf,'Nonlinear'); 

            FLFK = nlp.Plant.ContactPoints.FLfoot.computeCartesianPosition;  %  Front-Left Foot Forward Kinematics 
             Fz2 = FLFK(3);
          Fz2Fun = SymFunction('FLFootHeight',Fz2,{nlp.Plant.States.x}); 
          addNodeConstraint(nlp, Fz2Fun,{'x'},5:7,0.04,inf,'Nonlinear');
          
            RRFK = nlp.Plant.ContactPoints.RRfoot.computeCartesianPosition;  %  Rear-Right Foot Forward Kinematics 
             Fz3 = RRFK(3);
          Fz3Fun = SymFunction('RRFootHeight',Fz3,{nlp.Plant.States.x}); 
          addNodeConstraint(nlp, Fz3Fun,{'x'},5:7,0.04,inf,'Nonlinear');

            RLFK = nlp.Plant.ContactPoints.RLfoot.computeCartesianPosition;  %  Rear-Left Foot Forward Kinematics 
             Fz4 = RLFK(3); 
          Fz4Fun = SymFunction('RLFootHeight',Fz4,{nlp.Plant.States.x}); 
          addNodeConstraint(nlp,Fz4Fun,{'x'},5:7,0.04,inf,'Nonlinear');

%% Touchdown Velocity 
        addNodeConstraint(nlp, AllConstraints.SwingFootContactVelocity(nlp,'RL'), ...
        {'x','dx'}, 'last', [-1;-1;-1], [1;1;-0.0], 'Nonlinear');
    
%% Costs         
%       addRunningCost(nlp,AllCosts.Torque(nlp),'u');
      
end
