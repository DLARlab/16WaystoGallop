function [ Cost_of_Transport ] = COT( nlp ) % COT = Cost of Transport

%   Author: Yasser Alqaham
%     Date: 2020-04-21

%---------------------------------------------------------------------------------------------------------%

% Parameters %
             Mass = [4.713, 0.001, 0.696*4, 1.013*4, 0.166*4, 0.06*4]; % Masses of [Torso, IMU, Hip, Thigh, Calf, Foot]
       Total_Mass = sum(Mass); 
                g = 9.81;                                              % Gravitational acceleration
%---------------------------------------------------------------------------------------------------------%
                     
% Variables %
                         v = SymVariable('v');            % Average velocity 
 Torso_horizontal_velocity = nlp.Plant.States.dx(1);      % Body velocity
         joints_velocities = nlp.Plant.States.dx(7:18);   % Joints velocities 
                   torques = nlp.Plant.Inputs.Control.u;  % Joints torques (inputs)
%---------------------------------------------------------------------------------------------------------% 

% Cost Function = P/m.g.v (Cost of Transport) %                      
                      cost = sum(abs(torques.*joints_velocities))/(Total_Mass * g * Torso_horizontal_velocity);  
         Cost_of_Transport = SymFunction('COT', cost, {nlp.Plant.States.dx, nlp.Plant.Inputs.Control.u});
   
end

%---------------------------------------------------------------------------------------------------------%
% Cost_of_Transport = SymFunction('COT', cost, {nlp.Plant.States.dx, nlp.Plant.Inputs.Control.u}, {v});

                         % if Torso_horizontal_velocity == 0
                         %    Torso_horizontal_velocity = 0.1; % Eleminate dividing by zero
                         % end 