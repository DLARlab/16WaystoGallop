function contact = RL_toe_contact(robot)
    % The left hind toe contact point
    contact.frame = ToContactFrame(robot.ContactPoints.RLfoot,...
        'PointContactWithFriction');
    contact.fric_coef.mu = 0.6;
    contact.fric_coef.gamma = 100;
    contact.geometry.la = 0.05;
    contact.geometry.lb = 0.05;    
end