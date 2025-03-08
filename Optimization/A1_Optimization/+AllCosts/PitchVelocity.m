function [PitchVelocityFun] = PitchVelocity(nlp)
w = 1;
q_pitch_velocity = nlp.Plant.States.dx(5);
cost = {(w.*q_pitch_velocity).^2};
PitchVelocityFun = SymFunction('PitchVelocity', cost, {nlp.Plant.States.dx});
end

