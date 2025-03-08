function [COTdFun] = COTd(nlp) % COT = Cost of Transport
       mass = [4.713, 0.001, 0.696*4, 1.013*4, 0.166*4, 0.06*4]; % Masses of [Torso, IMU, Hip, Thigh, Calf, Foot]
       Mass = sum(mass); 
          g = 9.81;           
          T = cell(1,nlp.NumNode); 
      for i = 1:nlp.NumNode
       T{i} = SymVariable(['t',num2str(i)],[2,1]);     
      end
         x = cell(1,nlp.NumNode);
     for i = 1:nlp.NumNode
      x{i} = SymVariable(['x',num2str(i)],[nlp.Plant.numState,1]);   
      end
        dx = cell(1,nlp.NumNode);
     for i = 1:nlp.NumNode
     dx{i} = SymVariable(['dx',num2str(i)],[nlp.Plant.numState,1]);   
      end
         u = cell(1,nlp.NumNode);
     for i = 1:nlp.NumNode
      u{i} = SymVariable(['u',num2str(i)],[length(nlp.Plant.Inputs.Control.u),1]);   
      end
     Tstep = (T{2}(2) - T{1}(1))/(nlp.NumNode-1);
          w = [1;20;1;1;20;1;1;20;1;1;20;1];
 WorkStance = SymExpression(ones(nlp.NumNode,1));
      for i = 1:nlp.NumNode
WorkStance(i) = sum(sqrt((w.*u{i}.*dx{i}(7:18)).^2))*Tstep; 
      end
  TotalWorkStance = sum(WorkStance) - WorkStance(nlp.NumNode,1);
            COTd = (TotalWorkStance)/(Mass * g * (x{end}(1) - x{1}(1))); 
          COTdFun = SymFunction('COTd', COTd, [T,x,dx,u]); % use curly brackets to covert SymVar to cell arrays
                                         % Constr [T,H]    
end