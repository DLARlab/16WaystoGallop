function [ TorqueFun ] = Torque( nlp )
w = 1;
u = nlp.Plant.Inputs.Control.u;
cost = {sum((w.*u).^2)};
TorqueFun = SymFunction('Torque', cost, {u});
end

