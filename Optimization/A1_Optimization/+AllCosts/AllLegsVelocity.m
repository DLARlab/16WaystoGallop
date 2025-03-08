function [ AllLegsVelocityFun ] = AllLegsVelocity(nlp)
w = [1,1,1,1,1,1,1,1,1,1,1,1];
dq = nlp.Plant.States.dx(7:18);
cost = { (w(1)*dq(1)).^2   + (w(2)*dq(2)).^2   + (w(3)*dq(3)).^2 ...
       + (w(4)*dq(4)).^2   + (w(5)*dq(5)).^2   + (w(6)*dq(6)).^2 ...
       + (w(7)*dq(7)).^2   + (w(8)*dq(8)).^2   + (w(9)*dq(9)).^2 ... 
       + (w(10)*dq(10)).^2 + (w(11)*dq(11)).^2 + (w(12)*dq(12)).^2};
AllLegsVelocityFun = SymFunction('AllLegsVelocity', cost, {nlp.Plant.States.dx});
end

