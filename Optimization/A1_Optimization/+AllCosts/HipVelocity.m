function [HipVelocityFun] = HipVelocity( nlp )
w = 5;
q_hip_velocity = nlp.Plant.States.dx([7, 10, 13, 16]);
cost = {sum((w.*q_hip_velocity).^2)};
HipVelocityFun = SymFunction('HipVelocity', cost, {nlp.Plant.States.dx});
end

