function [HipRotationFun] = HipRotation( nlp )
w = 1;
q_hip_rotation = nlp.Plant.States.x([7,10,13,16]);
cost = {sum((w.*q_hip_rotation).^2)};
HipRotationFun = SymFunction('HipRotation', cost, {nlp.Plant.States.x});

end

