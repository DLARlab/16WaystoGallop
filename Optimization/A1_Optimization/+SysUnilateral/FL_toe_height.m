function h_nsf = FL_toe_height(robot)
    % left toe height constraint
    p_FL_toe = getCartesianPosition(robot, robot.ContactPoints.FLfoot);
    h_nsf = UnilateralConstraint(robot, p_FL_toe(3), 'FLtd', 'x');
end

 