function [RearTorqueFun] = RearTorque(nlp)
w = 1;
u = nlp.Plant.Inputs.Control.u(7:12);
cost = {sum((w.*u).^2)};
RearTorqueFun = SymFunction('RearTorque', cost, {nlp.Plant.Inputs.Control.u});
end

