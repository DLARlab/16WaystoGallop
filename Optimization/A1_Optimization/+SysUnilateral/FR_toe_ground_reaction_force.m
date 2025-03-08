function h_nsf = FR_toe_ground_reaction_force(robot)
% left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fFRfoot;
h_nsf = UnilateralConstraint(robot, GRF(3), 'FRlo', 'fFRfoot');

end

