function h_nsf = FL_toe_ground_reaction_force(robot)
% left toe ground reaction force
GRF = robot.Inputs.ConstraintWrench.fFLfoot;
h_nsf = UnilateralConstraint(robot, GRF(3), 'FLlo', 'fFLfoot');

end

