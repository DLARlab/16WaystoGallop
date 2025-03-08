function [ThighCalfVelocityFun] = ThighCalfVelocity( nlp )
w = [3,3,3,3,3,3,3,3];
q = nlp.Plant.States.dx([8,9,11,12,14,15,17,18]);
cost = {(w(1)*q(1)).^2 + (w(2)*q(2)).^2 + (w(3)*q(3)).^2 + (w(4)*q(4)).^2 + ...
        (w(5)*q(5)).^2 + (w(6)*q(6)).^2 + (w(7)*q(7)).^2 + (w(8)*q(8)).^2};
ThighCalfVelocityFun = SymFunction('ThighCalfVelocity', cost, {nlp.Plant.States.dx});
end

