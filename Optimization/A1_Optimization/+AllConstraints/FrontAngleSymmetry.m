function [FrontAngleSymmetryFun] = FrontAngleSymmetry(nlp)                
                          A = SymVariable('x',[nlp.Plant.numState,1]);
                          B = SymVariable('y',[nlp.Plant.numState,1]);                          
         FrontAngleSymmetry = [(A(8) + A(9)/2) + (B(8) + B(9)/2) ,(A1(11) + A1(12)/2) + (B1(11) + B1(12)/2)];              
      FrontAngleSymmetryFun = SymFunction('FrontAngleSymmetry', FrontAngleSymmetry, {A,B});
%                   D = SymVariable('x',[18,1]);
%                  FK = nlp.Plant.ContactPoints.FRfoot.computeForwardKinematics;               
%                 FA1 = subs(FK,D(:),A1(:));
%                 FB1 = subs(FK,D(:),B1(:));               
%                 Fx1 = subs(FR1(1,4),[D(1),D(2),D(3),D(4),D(5),D(6),D(7)],[0,0,0,0,0,0,0]);
%                 Fx2 = subs(FR2(1,4),[M(1),M(2),M(3),M(4),M(5),M(6),M(7)],[0,0,0,0,0,0,0]);      
%       SymmetryAngle = Fx1 + Fx2 - 2*0.183; % ------>
%      Symmetry Angle = 2*(ThetaThigh - Alpha) + (180 + ThetaCalf) %
%       SymmetryAngle = [(A1(8) + A1(9)/2); (A1(11) + A1(12)/2); (A1(14) + A1(15)/2); (A1(17) + A1(18)/2)] + ... 
%                       [(B1(8) + B1(9)/2); (B1(11) + B1(12)/2); (B1(14) + B1(15)/2); (B1(17) + B1(18)/2)]; 

end
