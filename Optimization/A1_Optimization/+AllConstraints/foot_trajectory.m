function [ foot_trajectory_fun ] = foot_trajectory( nlp, i )
   
%   Author:  
%     Date: 

         z1  = 0.07; % Maximum hight of the foot
         T   = SymVariable('t',[2,1]);
        Tmid = (T(2) - T(1)) / 2;
       Tstep = (T(2) - T(1))/nlp.NumNode;
            % 4 by 4 homogenuous transformation matrix

% Fitting a parabola to three points %

           A = [(T(1)).^2, T(1), 1; Tmid.^2, Tmid, 1; (T(2)).^2, T(2), 1]; 
           B = [0; z1; 0];
           X = A\B;   %  X(1) = a,   X(2) = b,   X(3) = c  
   
% Creating the constraint equation % 

     Q = cell(1,nlp.NumNode); % cell arry that has cells equals to the number of nodes. 
Constr = SymExpression(ones(nlp.NumNode,1));

for i = 1: nlp.NumNode
    
              FK = nlp.Plant.ContactPoints.FRfoot.computeForwardKinematics;  % 4 by 4 homogenuous transformation matrix 
           Q{i}  = SymVariable('x',[nlp.Plant.numState,1]);
            Time =  T(1)+i*Tstep;
     Constr(i,1) = X(1)*(Time.^2) + X(2)*(Time)+ X(3) - FK(3,4); 
          
end

foot_trajectory_fun  = SymFunction('foot_trajectory', Constr, {T,nlp.Plant.States.x}); % use curly brackets to covert SymVar to cell arrays
                                                        
end
                                                        % [{T},Q] , {T,nlp.Plant.States.x}



%    Q = cell(1,11); 
% Constr = SymExpression(ones(11,1));
% 
% for i = 1 : 11
%     
%               FK = nlp.Phase(3).Plant.ContactPoints.FRfoot.computeForwardKinematics;  
%            Q{i}  = SymVariable('x',[18,1]);
%             Time =  T(1)+i*Tstep;
%      Constr(i,1) = X(1)*(Time.^2) + X(2)*(Time)+ X(3) - FK(3,4); 
%           
% end




