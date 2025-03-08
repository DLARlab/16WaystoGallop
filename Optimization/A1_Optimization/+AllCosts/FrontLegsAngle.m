function [FrontLegsAngleFun] = FrontLegsAngle( nlp )
w = 100;
q = nlp.Plant.States.x([8,9,14,15]);
cost = {(w.*(q(1) + q(2)/2)).^2 + (w.*(q(3) + q(4)/2)).^2 };
FrontLegsAngleFun = SymFunction('FrontLegsAngle', cost, {nlp.Plant.States.x});
end

