function [SwingFootContactVelocityFun] = SwingFootContactVelocity(nlp,foot)

switch foot
    
     case 'FR'
        swing_foot = nlp.Plant.ContactPoints.FRfoot;
     case 'FL'
        swing_foot = nlp.Plant.ContactPoints.FLfoot;  
     case 'RR'
        swing_foot = nlp.Plant.ContactPoints.RRfoot;   
     case 'RL'
        swing_foot = nlp.Plant.ContactPoints.RLfoot;
     otherwise
         error('Cannot create swing_foot_contact_velocity constraint. Unknown foot')
end

     x =  nlp.Plant.States.x;
    dx =  nlp.Plant.States.dx;
     
      P = swing_foot.computeCartesianPosition;  
      J = jacobian(P, x);
      V = J * dx;

SwingFootContactVelocityFun = SymFunction(['SwingFootContactVelocity_',foot], V, {x,dx});

end
%% Old Code (Correct)
% switch foot
%     
%      case 'FR'
%         swing_foot = nlp.Plant.ContactPoints.FRfoot;
%                  R = nlp.Plant.ContactPoints.FRfoot.computeForwardKinematics;
%               AugR = [R(1:3,1:3),zeros(3,3);zeros(3,3),eye(3,3)];     
%           FootName = 'FR';
%      case 'FL'
%         swing_foot = nlp.Plant.ContactPoints.FLfoot;
%                  R = nlp.Plant.ContactPoints.FLfoot.computeForwardKinematics;
%               AugR = [R(1:3,1:3),zeros(3,3);zeros(3,3),eye(3,3)];     
%           FootName = 'FL';
%      case 'RR'
%         swing_foot = nlp.Plant.ContactPoints.RRfoot;
%                  R = nlp.Plant.ContactPoints.RRfoot.computeForwardKinematics;
%               AugR = [R(1:3,1:3),zeros(3,3);zeros(3,3),eye(3,3)];     
%           FootName = 'RR';
%      case 'RL'
%         swing_foot = nlp.Plant.ContactPoints.RLfoot;
%                  R = nlp.Plant.ContactPoints.RLfoot.computeForwardKinematics;
%               AugR = [R(1:3,1:3),zeros(3,3);zeros(3,3),eye(3,3)];     
%           FootName = 'RL';
%      otherwise
%          error('Cannot create swing_foot_contact_velocity constraint. Unknown foot')
%  end
% 
% J_swfoot = swing_foot.computeBodyJacobian(nlp.Plant.numState);
% v_swfoot =  AugR * (J_swfoot * nlp.Plant.States.dx);
% SwingFootContactVelocityFun = SymFunction(['SwingFootContactVelocity_',FootName], v_swfoot, {nlp.Plant.States.x, nlp.Plant.States.dx});

