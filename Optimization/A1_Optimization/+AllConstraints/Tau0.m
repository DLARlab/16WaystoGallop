function [Tau0Fun] = Tau0(nlp)
T  = SymVariable('t',[2,1]);
p = {SymVariable(tomatrix(nlp.Plant.VirtualConstraints.time.PhaseParams(:)))};
Tau0Fun = SymFunction(['Tau0_',nlp.Plant.Name], T(1) - p{1}(2), [{T},p]);
end

