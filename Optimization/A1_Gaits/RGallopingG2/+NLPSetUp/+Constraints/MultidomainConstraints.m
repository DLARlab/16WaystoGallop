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

%% COTdisHS 
      mass = [4.713, 0.001, 0.696*4, 1.013*4, 0.166*4, 0.06*4]; % Masses of [Torso, IMU, Hip, Thigh, Calf, Foot]
      Mass = sum(mass); 
         g = 9.81; 
 numInputs = 12;
% Time ----------------------------------------------------------------------------------------------------------               
        t1 = cell(1,nlp.Phase(1).NumNode); 
     for i = 1:nlp.Phase(1).NumNode
     t1{i} = SymVariable(['t1',num2str(i)],[2,1]);     
     end
     
        t2 = cell(1,nlp.Phase(3).NumNode); 
     for i = 1:nlp.Phase(3).NumNode
     t2{i} = SymVariable(['t2',num2str(i)],[2,1]);     
     end
       
        t3 = cell(1,nlp.Phase(5).NumNode); 
     for i = 1:nlp.Phase(5).NumNode
     t3{i} = SymVariable(['t3',num2str(i)],[2,1]);     
     end
 
        t4 = cell(1,nlp.Phase(7).NumNode); 
     for i = 1:nlp.Phase(7).NumNode
     t4{i} = SymVariable(['t4',num2str(i)],[2,1]);     
     end  
     
        t5 = cell(1,nlp.Phase(9).NumNode); 
     for i = 1:nlp.Phase(9).NumNode
     t5{i} = SymVariable(['t5',num2str(i)],[2,1]);     
     end  
     
        t6 = cell(1,nlp.Phase(11).NumNode); 
     for i = 1:nlp.Phase(11).NumNode
     t6{i} = SymVariable(['t6',num2str(i)],[2,1]);     
     end  
     
        t7 = cell(1,nlp.Phase(13).NumNode); 
     for i = 1:nlp.Phase(13).NumNode
     t7{i} = SymVariable(['t7',num2str(i)],[2,1]);     
     end  
     
        t8 = cell(1,nlp.Phase(15).NumNode); 
     for i = 1:nlp.Phase(15).NumNode
     t8{i} = SymVariable(['t8',num2str(i)],[2,1]);     
     end  
% Configuration -------------------------------------------------------------------------------------------------        
        x1 = cell(1,nlp.Phase(1).NumNode);
     for i = 1:nlp.Phase(1).NumNode
     x1{i} = SymVariable(['x1',num2str(i)],[nlp.Phase(1).Plant.numState,1]);   
     end
      
        x8 = cell(1,nlp.Phase(15).NumNode);
     for i = 1:nlp.Phase(15).NumNode
     x8{i} = SymVariable(['x8',num2str(i)],[nlp.Phase(15).Plant.numState,1]);   
     end     
% Velocities ----------------------------------------------------------------------------------------------------       
       dx1 = cell(1,nlp.Phase(1).NumNode);
     for i = 1:nlp.Phase(1).NumNode
    dx1{i} = SymVariable(['dx1',num2str(i)],[nlp.Phase(1).Plant.numState,1]);   
     end
     
       dx2 = cell(1,nlp.Phase(3).NumNode);
     for i = 1:nlp.Phase(3).NumNode
    dx2{i} = SymVariable(['dx2',num2str(i)],[nlp.Phase(3).Plant.numState,1]);   
     end
      
       dx3 = cell(1,nlp.Phase(5).NumNode);
     for i = 1:nlp.Phase(5).NumNode
    dx3{i} = SymVariable(['dx3',num2str(i)],[nlp.Phase(5).Plant.numState,1]);   
     end
      
       dx4 = cell(1,nlp.Phase(7).NumNode);
     for i = 1:nlp.Phase(7).NumNode
    dx4{i} = SymVariable(['dx4',num2str(i)],[nlp.Phase(7).Plant.numState,1]);   
     end  
     
       dx5 = cell(1,nlp.Phase(9).NumNode);
     for i = 1:nlp.Phase(9).NumNode
    dx5{i} = SymVariable(['dx5',num2str(i)],[nlp.Phase(9).Plant.numState,1]);   
     end   
     
       dx6 = cell(1,nlp.Phase(11).NumNode);
     for i = 1:nlp.Phase(11).NumNode
    dx6{i} = SymVariable(['dx6',num2str(i)],[nlp.Phase(11).Plant.numState,1]);   
     end   
     
       dx7 = cell(1,nlp.Phase(13).NumNode);
     for i = 1:nlp.Phase(13).NumNode
    dx7{i} = SymVariable(['dx7',num2str(i)],[nlp.Phase(13).Plant.numState,1]);   
     end   
     
       dx8 = cell(1,nlp.Phase(15).NumNode);
     for i = 1:nlp.Phase(15).NumNode
    dx8{i} = SymVariable(['dx8',num2str(i)],[nlp.Phase(15).Plant.numState,1]);   
     end   
% Input ---------------------------------------------------------------------------------------------------------      
        u1 = cell(1,nlp.Phase(1).NumNode);
     for i = 1:nlp.Phase(1).NumNode
     u1{i} = SymVariable(['u1',num2str(i)],[numInputs,1]);   
     end
     
        u2 = cell(1,nlp.Phase(3).NumNode);
     for i = 1:nlp.Phase(3).NumNode
     u2{i} = SymVariable(['u2',num2str(i)],[numInputs,1]);   
     end
      
        u3 = cell(1,nlp.Phase(5).NumNode);
     for i = 1:nlp.Phase(5).NumNode
     u3{i} = SymVariable(['u3',num2str(i)],[numInputs,1]);   
     end
      
        u4 = cell(1,nlp.Phase(7).NumNode);
     for i = 1:nlp.Phase(7).NumNode
     u4{i} = SymVariable(['u4',num2str(i)],[numInputs,1]);   
     end    
     
        u5 = cell(1,nlp.Phase(9).NumNode);
     for i = 1:nlp.Phase(9).NumNode
     u5{i} = SymVariable(['u5',num2str(i)],[numInputs,1]);   
     end     
     
        u6 = cell(1,nlp.Phase(11).NumNode);
     for i = 1:nlp.Phase(11).NumNode
     u6{i} = SymVariable(['u6',num2str(i)],[numInputs,1]);   
     end     
     
        u7 = cell(1,nlp.Phase(13).NumNode);
     for i = 1:nlp.Phase(13).NumNode
     u7{i} = SymVariable(['u7',num2str(i)],[numInputs,1]);   
     end     
     
        u8 = cell(1,nlp.Phase(15).NumNode);
     for i = 1:nlp.Phase(15).NumNode
     u8{i} = SymVariable(['u8',num2str(i)],[numInputs,1]);   
     end     
% Constarint Expression---------------------------------------------------------------------------------------------------------------
       T1 = (t1{2}(2) - t1{1}(1));
       T2 = (t2{2}(2) - t2{1}(1));
       T3 = (t3{2}(2) - t3{1}(1));
       T4 = (t4{2}(2) - t4{1}(1));
       T5 = (t5{2}(2) - t5{1}(1));
       T6 = (t6{2}(2) - t6{1}(1));
       T7 = (t7{2}(2) - t7{1}(1));
       T8 = (t8{2}(2) - t8{1}(1));   
        w = diag([1,1,1,1,1,1,1,1,1,1,1,1]); 
    
        n = 0;
    nGrid = floor(nlp.Phase(1).NumNode/2);
    Work1 = SymExpression(ones(nGrid,1));
    for i = 1:nGrid
         Work1(i,1) = (T1./(2*nGrid)).*(1/3).*(sum(sqrt(((u1{i+n}.*dx1{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u1{i+n+1}.*dx1{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u1{i+n+2}.*dx1{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;
    end     
         TotalWork1 = sum(Work1);
         
        n = 0;
    nGrid = floor(nlp.Phase(3).NumNode/2);
    Work2 = SymExpression(ones(nGrid,1));
     for i = 1:nGrid
         Work2(i) = (T2./(2*nGrid)).*(1/3).*(sum(sqrt(((u2{i+n}.*dx2{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u2{i+n+1}.*dx2{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u2{i+n+2}.*dx2{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;
    end       
         TotalWork2 = sum(Work2);
         
        n = 0;
    nGrid = floor(nlp.Phase(5).NumNode/2);
    Work3 = SymExpression(ones(nGrid,1));
    for i = 1:nGrid
         Work3(i) = (T3./(2*nGrid)).*(1/3).*(sum(sqrt(((u3{i+n}.*dx3{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u3{i+n+1}.*dx3{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u3{i+n+2}.*dx3{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;      
    end     
         TotalWork3 = sum(Work3);
         
        n = 0;
    nGrid = floor(nlp.Phase(7).NumNode/2);
    Work4 = SymExpression(ones(nGrid,1));
    for i = 1:nGrid
         Work4(i) = (T4./(2*nGrid)).*(1/3).*(sum(sqrt(((u4{i+n}.*dx4{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u4{i+n+1}.*dx4{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u4{i+n+2}.*dx4{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;      
    end     
         TotalWork4 = sum(Work4);
 
         
        n = 0;
    nGrid = floor(nlp.Phase(9).NumNode/2);
    Work5 = SymExpression(ones(nGrid,1));
    for i = 1:nGrid
         Work5(i) = (T5./(2*nGrid)).*(1/3).*(sum(sqrt(((u5{i+n}.*dx5{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u5{i+n+1}.*dx5{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u5{i+n+2}.*dx5{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;      
    end     
         TotalWork5 = sum(Work5);
         

        n = 0;
    nGrid = floor(nlp.Phase(11).NumNode/2);
    Work6 = SymExpression(ones(nGrid,1));
    for i = 1:nGrid
         Work6(i) = (T6./(2*nGrid)).*(1/3).*(sum(sqrt(((u6{i+n}.*dx6{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u6{i+n+1}.*dx6{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u6{i+n+2}.*dx6{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;      
    end     
         TotalWork6 = sum(Work6);
         

        n = 0;
    nGrid = floor(nlp.Phase(13).NumNode/2);
    Work7 = SymExpression(ones(nGrid,1));
    for i = 1:nGrid
         Work7(i) = (T7./(2*nGrid)).*(1/3).*(sum(sqrt(((u7{i+n}.*dx7{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u7{i+n+1}.*dx7{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u7{i+n+2}.*dx7{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;      
    end     
         TotalWork7 = sum(Work7);
         
       
        n = 0;
    nGrid = floor(nlp.Phase(15).NumNode/2);
    Work8 = SymExpression(ones(nGrid,1));
    for i = 1:nGrid
         Work8(i) = (T8./(2*nGrid)).*(1/3).*(sum(sqrt(((u8{i+n}.*dx8{i+n}(7:18)).^2) + 0.00001)) + 4*sum(sqrt(((u8{i+n+1}.*dx8{i+n+1}(7:18)).^2) + 0.00001)) + sum(sqrt(((u8{i+n+2}.*dx8{i+n+2}(7:18)).^2) + 0.00001)));      % (dx1{i}(7:18)*w*u1{i})*Tstep1*weight;------ if w = diag([1,1,1,1,1,1,1,1,1,1,1,1]) is added.
                n = n+1;      
    end     
         TotalWork8 = sum(Work8);
         
                  
         
 TotalWork = TotalWork1 + TotalWork2 + TotalWork3 + TotalWork4 + TotalWork5 + TotalWork6 + TotalWork7 + TotalWork8;
 
      dCOT = (TotalWork)/(Mass * g * (x8{end}(1) - x1{1}(1)));  
      
  dCOT_fun = SymFunction('COTdisHS',dCOT,[t1,t2,t3,t4,t5,t6,t7,t8,x1,x8,dx1,dx2,dx3,dx4,dx5,dx6,dx7,dx8,u1,u2,u3,u4,u5,u6,u7,u8]); % use curly brackets to covert SymVar to cell arrays
                                                               % Constr   [T,H]
 dCOT_cstrnt = NlpFunction('Name','COTdisHS',...
                           'Dimension',1,...
                           'lb', -inf,...
                           'ub', +inf,...
                           'Type','Nonlinear',...
                           'SymFun',dCOT_fun,...
                           'DepVariables',[nlp.Phase(1).OptVarTable.T', nlp.Phase(3).OptVarTable.T', nlp.Phase(5).OptVarTable.T', nlp.Phase(7).OptVarTable.T',...
                                           nlp.Phase(9).OptVarTable.T', nlp.Phase(11).OptVarTable.T', nlp.Phase(13).OptVarTable.T', nlp.Phase(15).OptVarTable.T',...
                                           nlp.Phase(1).OptVarTable.x', nlp.Phase(15).OptVarTable.x',...
                                           nlp.Phase(1).OptVarTable.dx',nlp.Phase(3).OptVarTable.dx',nlp.Phase(5).OptVarTable.dx',nlp.Phase(7).OptVarTable.dx',...
                                           nlp.Phase(9).OptVarTable.dx',nlp.Phase(11).OptVarTable.dx',nlp.Phase(13).OptVarTable.dx',nlp.Phase(15).OptVarTable.dx',...
                                           nlp.Phase(1).OptVarTable.u', nlp.Phase(3).OptVarTable.u', nlp.Phase(5).OptVarTable.u', nlp.Phase(7).OptVarTable.u',...
                                           nlp.Phase(9).OptVarTable.u', nlp.Phase(11).OptVarTable.u', nlp.Phase(13).OptVarTable.u', nlp.Phase(15).OptVarTable.u']); 
         
      addCost(nlp.Phase(15), 'COTdisHS', 'last', dCOT_cstrnt);
      
end
