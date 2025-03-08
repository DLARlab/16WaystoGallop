function [CalfVelocityFun] = CalfVelocity( nlp )
w = 0.1;
q_calf_velocity = nlp.Plant.States.dx([9,12,15,18]);
cost = {sum((w.*q_calf_velocity).^2)};
CalfVelocityFun = SymFunction('CalfVelocity', cost, {nlp.Plant.States.dx});
end