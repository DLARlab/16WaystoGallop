function [TorsoHeightFun] = TorsoHeight(nlp) 
            h  = SymVariable('h',[nlp.Plant.numState,1]); 
        h_apex = h(3); 
TorsoHeightFun = SymFunction('TorsoHeihgt', h_apex, {h});
end 