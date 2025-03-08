function h_nsf = RR_toe_height(robot)
    % left toe height constraint
    p_RR_toe = getCartesianPosition(robot, robot.ContactPoints.RRfoot);
    h_nsf = UnilateralConstraint(robot, p_RR_toe(3), 'RRtd', 'x');
end

 