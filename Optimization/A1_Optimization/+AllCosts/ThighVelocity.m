function [ThighVelocityFun] = ThighVelocity(nlp)
w = 1;
q_thigh_velocity = nlp.Plant.States.dx([8,11,14,17]);
cost = {sum((w.*q_thigh_velocity).^2)};
ThighVelocityFun = SymFunction('ThighVelocity', cost, {nlp.Plant.States.dx});
end

