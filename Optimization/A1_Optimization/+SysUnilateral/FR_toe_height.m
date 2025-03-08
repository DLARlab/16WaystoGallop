function h_nsf = FR_toe_height(robot)
    % left toe height constraint
    p_FR_toe = getCartesianPosition(robot, robot.ContactPoints.FRfoot);
    h_nsf = UnilateralConstraint(robot, p_FR_toe(3), 'FRtd', 'x');
end

 