function [TauFFun] = TauF(nlp)
T  = SymVariable('t',[2,1]);
p = {SymVariable(tomatrix(nlp.Plant.VirtualConstraints.time.PhaseParams(:)))};
TauFFun = SymFunction(['TauF_',nlp.Plant.Name], T(2) - p{1}(1), [{T},p]);

end

