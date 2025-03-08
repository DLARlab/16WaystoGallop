function [FrontTorqueFun] = FrontTorque(nlp)
w = 1;
u = nlp.Plant.Inputs.Control.u(1:6);
cost = {sum((w.*u).^2)};
FrontTorqueFun = SymFunction('FrontTorque', cost, {nlp.Plant.Inputs.Control.u});
end
