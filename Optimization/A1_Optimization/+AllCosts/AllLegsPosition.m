function [AllLegsPositionFun] = AllLegsPosition(nlp)
w = [3,3,3,3,3,3,3,3,3,3,3,3];
q = nlp.Plant.States.x(7:18);
cost = { (w(1)*q(1)).^2   + (w(2)*q(2)).^2   + (w(3)*q(3)).^2 ...
       + (w(4)*q(4)).^2   + (w(5)*q(5)).^2   + (w(6)*q(6)).^2 ...
       + (w(7)*q(7)).^2   + (w(8)*q(8)).^2   + (w(9)*q(9)).^2 ... 
       + (w(10)*q(10)).^2 + (w(11)*q(11)).^2 + (w(12)*q(12)).^2};
AllLegsPositionFun = SymFunction('AllLegsPosition', cost, {nlp.Plant.States.x});
end

