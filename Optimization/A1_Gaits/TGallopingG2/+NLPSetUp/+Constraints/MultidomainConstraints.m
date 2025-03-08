function [nlp] = MultidomainConstraints(nlp,bounds)
% Average Velocity
    T1 = SymVariable('t1',[2,1]);
    T2 = SymVariable('t2',[2,1]);
    T3 = SymVariable('t3',[2,1]);
    T4 = SymVariable('t4',[2,1]);
    T5 = SymVariable('t5',[2,1]);
    T6 = SymVariable('t6',[2,1]);
    T7 = SymVariable('t7',[2,1]);
    T8 = SymVariable('t8',[2,1]);
    X0 = SymVariable('x0',[nlp.Phase(1).Plant.numState,1]);
    XF = SymVariable('xF',[nlp.Phase(1).Plant.numState,1]);
    
    avg_vel = (XF(1:2) - X0(1:2)) ./ (T1(2) - T1(1) + T2(2) - T2(1) + T3(2) - T3(1) + T4(2) - T4(1) + T5(2) - T5(1) + T6(2) - T6(1) + T7(2) - T7(1) + T8(2) - T8(1));
    avg_vel_fun = SymFunction('average_velocity', avg_vel, {T1,T2,T3,T4,T5,T6,T7,T8,X0,XF}); % use curly brackets to covert SymVar to cell arrays
    
    average_velocity_cstr = NlpFunction('Name',['average_velocity_','entire_stride'],...
                                        'Dimension',2,...
                                        'lb', bounds.average_velocity.lb,...
                                        'ub', bounds.average_velocity.ub ,...
                                        'Type','Nonlinear',...
                                        'SymFun',avg_vel_fun,...
                                        'DepVariables',[nlp.Phase(1).OptVarTable.T(1),nlp.Phase(3).OptVarTable.T(1),...
                                                        nlp.Phase(5).OptVarTable.T(1),nlp.Phase(7).OptVarTable.T(1),...
                                                        nlp.Phase(9).OptVarTable.T(1),nlp.Phase(11).OptVarTable.T(1),...
                                                        nlp.Phase(13).OptVarTable.T(1),nlp.Phase(15).OptVarTable.T(1),...
                                                        nlp.Phase(1).OptVarTable.x(1),nlp.Phase(15).OptVarTable.x(end)]);
    
    addConstraint(nlp.Phase(15), ['average_velocity_','entire_stride'], 'last', average_velocity_cstr);

 %% Average Velocity
%     T1 = SymVariable('t1',[2,1]);
%     T2 = SymVariable('t2',[2,1]);
%     T3 = SymVariable('t3',[2,1]);
%     T4 = SymVariable('t4',[2,1]);
% 
%     X0 = SymVariable('x0',[nlp.Phase(1).Plant.numState,1]);
%     XF = SymVariable('xF',[nlp.Phase(1).Plant.numState,1]);
%     
%     avg_vel = (XF(1:2) - X0(1:2)) ./ (T1(2) - T1(1) + T2(2) - T2(1) + T3(2) - T3(1) + T4(2) - T4(1));
%     avg_vel_fun = SymFunction('average_velocity', avg_vel, {T1,T2,T3,T4,X0,XF}); % use curly brackets to covert SymVar to cell arrays
%     
%     average_velocity_cstr = NlpFunction('Name',['average_velocity_','entire_stride'],...
%                                         'Dimension',2,...
%                                         'lb', bounds.average_velocity.lb,...
%                                         'ub', bounds.average_velocity.ub ,...
%                                         'Type','Nonlinear',...
%                                         'SymFun',avg_vel_fun,...
%                                         'DepVariables',[nlp.Phase(1).OptVarTable.T(1),nlp.Phase(3).OptVarTable.T(1),...
%                                                         nlp.Phase(5).OptVarTable.T(1),nlp.Phase(7).OptVarTable.T(1),...
%                                                         nlp.Phase(1).OptVarTable.x(1),nlp.Phase(7).OptVarTable.x(end)]);
%     
%     addConstraint(nlp.Phase(7), ['average_velocity_','entire_stride'], 'last', average_velocity_cstr);
    
%% dCOT 
%       mass = [4.713, 0.001, 0.696*4, 1.013*4, 0.166*4, 0.06*4]; % Masses of [Torso, IMU, Hip, Thigh, Calf, Foot]
%       Mass = sum(mass); 
%          g = 9.81;  
% % Time ----------------------------------------------------------------------------------------------------------               
%        tFS = cell(1,nlp.Phase(1).NumNode); 
%      for i = 1:nlp.Phase(1).NumNode
%     tFS{i} = SymVariable(['tFS',num2str(i)],[2,1]);     
%      end
%      
%        tFF = cell(1,nlp.Phase(3).NumNode); 
%      for i = 1:nlp.Phase(3).NumNode
%     tFF{i} = SymVariable(['tFF',num2str(i)],[2,1]);     
%      end
%        
%        tRS = cell(1,nlp.Phase(5).NumNode); 
%      for i = 1:nlp.Phase(5).NumNode
%     tRS{i} = SymVariable(['tRS',num2str(i)],[2,1]);     
%      end
%  
%        tRF = cell(1,nlp.Phase(7).NumNode); 
%      for i = 1:nlp.Phase(7).NumNode
%     tRF{i} = SymVariable(['tRF',num2str(i)],[2,1]);     
%      end
%       
% % Configuration -------------------------------------------------------------------------------------------------        
%        xFS = cell(1,nlp.Phase(1).NumNode);
%      for i = 1:nlp.Phase(1).NumNode
%     xFS{i} = SymVariable(['xFS',num2str(i)],[nlp.Phase(1).Plant.numState,1]);   
%      end
%      
%        xFF = cell(1,nlp.Phase(3).NumNode);
%      for i = 1:nlp.Phase(3).NumNode
%     xFF{i} = SymVariable(['xFF',num2str(i)],[nlp.Phase(3).Plant.numState,1]);   
%      end
%       
%        xRS = cell(1,nlp.Phase(5).NumNode);
%      for i = 1:nlp.Phase(5).NumNode
%     xRS{i} = SymVariable(['xRS',num2str(i)],[nlp.Phase(5).Plant.numState,1]);   
%      end
%       
%        xRF = cell(1,nlp.Phase(7).NumNode);
%      for i = 1:nlp.Phase(7).NumNode
%     xRF{i} = SymVariable(['xRF',num2str(i)],[nlp.Phase(7).Plant.numState,1]);   
%      end
%       
% % Velocities ----------------------------------------------------------------------------------------------------       
%       dxFS = cell(1,nlp.Phase(1).NumNode);
%      for i = 1:nlp.Phase(1).NumNode
%    dxFS{i} = SymVariable(['dxFS',num2str(i)],[nlp.Phase(1).Plant.numState,1]);   
%      end
%      
%       dxFF = cell(1,nlp.Phase(3).NumNode);
%      for i = 1:nlp.Phase(3).NumNode
%    dxFF{i} = SymVariable(['dxFF',num2str(i)],[nlp.Phase(3).Plant.numState,1]);   
%      end
%       
%       dxRS = cell(1,nlp.Phase(5).NumNode);
%      for i = 1:nlp.Phase(5).NumNode
%    dxRS{i} = SymVariable(['dxRS',num2str(i)],[nlp.Phase(5).Plant.numState,1]);   
%      end
%       
%       dxRF = cell(1,nlp.Phase(7).NumNode);
%      for i = 1:nlp.Phase(7).NumNode
%    dxRF{i} = SymVariable(['dxRF',num2str(i)],[nlp.Phase(7).Plant.numState,1]);   
%      end       
% 
% % Input ---------------------------------------------------------------------------------------------------------      
%         uFS = cell(1,nlp.Phase(1).NumNode);
%       for i = 1:nlp.Phase(1).NumNode
%      uFS{i} = SymVariable(['uFS',num2str(i)],[length(nlp.Phase(1).Plant.Inputs.Control.u),1]);   
%       end
%      
%         uFF = cell(1,nlp.Phase(3).NumNode);
%       for i = 1:nlp.Phase(3).NumNode
%      uFF{i} = SymVariable(['uFF',num2str(i)],[length(nlp.Phase(3).Plant.Inputs.Control.u),1]);   
%       end
%       
%        uRS = cell(1,nlp.Phase(5).NumNode);
%      for i = 1:nlp.Phase(5).NumNode
%     uRS{i} = SymVariable(['uRS',num2str(i)],[length(nlp.Phase(5).Plant.Inputs.Control.u),1]);   
%      end
%       
%        uRF = cell(1,nlp.Phase(7).NumNode);
%      for i = 1:nlp.Phase(7).NumNode
%     uRF{i} = SymVariable(['uRF',num2str(i)],[length(nlp.Phase(7).Plant.Inputs.Control.u),1]);   
%      end
%       
% % Constarint Expression---------------------------------------------------------------------------------------------------------------
%     Tstep1 = (tFS{2}(2) - tFS{1}(1))/(nlp.Phase(1).NumNode-1);
%     Tstep2 = (tFF{2}(2) - tFS{1}(1))/(nlp.Phase(3).NumNode-1);
%     Tstep3 = (tRS{2}(2) - tFS{1}(1))/(nlp.Phase(5).NumNode-1);
%     Tstep4 = (tRF{2}(2) - tFS{1}(1))/(nlp.Phase(7).NumNode-1);
%                
%          w = [1;1;1;1;1;1;1;1;1;1;1;1];
%                 
%     WorkFS = SymExpression(ones(nlp.Phase(1).NumNode,1));
%      for i = 1:nlp.Phase(1).NumNode-1
%  WorkFS(i) = sum(sqrt((w.*uFS{i}.*dxFS{i}(7:18)).^2))*Tstep1; 
%      end
% TotalWorkFS = sum(WorkFS);
%    
%     WorkFF = SymExpression(ones(nlp.Phase(3).NumNode,1));
%      for i = 1:nlp.Phase(3).NumNode-1
%  WorkFF(i) = sum(sqrt((w.*uFF{i}.*dxFF{i}(7:18)).^2))*Tstep2; 
%      end
% TotalWorkFF = sum(WorkFF);
%   
%     WorkRS = SymExpression(ones(nlp.Phase(5).NumNode,1));
%      for i = 1:nlp.Phase(5).NumNode-1
%  WorkRS(i) = sum(sqrt((w.*uRS{i}.*dxRS{i}(7:18)).^2))*Tstep3; 
%      end
% TotalWorkRS = sum(WorkRS);
%    
%     WorkRF = SymExpression(ones(nlp.Phase(7).NumNode,1));
%      for i = 1:nlp.Phase(7).NumNode-1
%  WorkRF(i) = sum(sqrt((w.*uRF{i}.*dxRF{i}(7:18)).^2))*Tstep4; 
%      end
% TotalWorkRF = sum(WorkRF);
%   
%  
%  TotalWork = TotalWorkFS + TotalWorkFF + TotalWorkRS + TotalWorkRF;
%       dCOT = (TotalWork)/(Mass * g * (xRF{end}(1) - xFS{1}(1)));       
%          
%   dCOT_fun = SymFunction('dCOTall',dCOT,[tFS,tFF,tRS,tRF,xFS,xFF,xRS,xRF,dxFS,dxFF,dxRS,dxRF,uFS,uFF,uRS,uRF]); % use curly brackets to covert SymVar to cell arrays
%                                                                % Constr   [T,H]
% 
%  dCOT_cost = NlpFunction('Name','dCOTall',...
%                          'Dimension',1,...
%                          'lb', -inf,...
%                          'ub', +inf,...
%                          'Type','Nonlinear',...
%                          'SymFun',dCOT_fun,...
%                          'DepVariables',[nlp.Phase(1).OptVarTable.T',nlp.Phase(3).OptVarTable.T',nlp.Phase(5).OptVarTable.T',nlp.Phase(7).OptVarTable.T',...
%                                          nlp.Phase(1).OptVarTable.x',nlp.Phase(3).OptVarTable.x',nlp.Phase(5).OptVarTable.x',nlp.Phase(7).OptVarTable.x',...
%                                          nlp.Phase(1).OptVarTable.dx',nlp.Phase(3).OptVarTable.dx',nlp.Phase(5).OptVarTable.dx',nlp.Phase(7).OptVarTable.dx',...
%                                          nlp.Phase(1).OptVarTable.u',nlp.Phase(3).OptVarTable.u',nlp.Phase(5).OptVarTable.u',nlp.Phase(7).OptVarTable.u']); 
%          
%      addCost(nlp.Phase(7), 'dCOTall', 'last', dCOT_cost);
       
end
