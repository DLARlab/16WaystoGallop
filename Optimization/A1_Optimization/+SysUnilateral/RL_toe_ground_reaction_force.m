function h_nsf = RL_toe_ground_reaction_force(robot)
% left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fRLfoot;
h_nsf = UnilateralConstraint(robot, GRF(3), 'RLlo', 'fRLfoot');

end

