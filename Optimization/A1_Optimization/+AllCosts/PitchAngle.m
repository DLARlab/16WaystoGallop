function [PitchAngleFun] = PitchAngle(nlp)
w = 1;
q_pitch_angle = nlp.Plant.States.x(5);
cost = {(w.*q_pitch_angle).^2};
PitchAngleFun = SymFunction('PitchAngle', cost, {nlp.Plant.States.x});
end

