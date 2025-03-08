function h_nsf = RR_toe_ground_reaction_force(robot)
% left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fRRfoot;
h_nsf = UnilateralConstraint(robot, GRF(3), 'RRlo', 'fRRfoot');

end

