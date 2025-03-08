function h_nsf = RL_toe_height(robot)
    % left toe height constraint
    p_RL_toe = getCartesianPosition(robot, robot.ContactPoints.RLfoot);
    h_nsf = UnilateralConstraint(robot, p_RL_toe(3), 'RLtd', 'x');
end

 