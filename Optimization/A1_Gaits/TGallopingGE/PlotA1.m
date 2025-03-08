
function  PlotA1(gait,what)
      Phase_Time = [gait.tspan];
            Time = cell2mat(Phase_Time);      % Stride Time %
number_of_phases = size([gait.tspan],2);      % Number of Phases %
               v =  round((gait(end - 1).states.x(1,11) - gait(1).states.x(1,1))/gait(end - 1).tspan(1,11));
    n = 0;    
for i = 1 : number_of_phases    
%---------------------------Body------------------------------------%
       Body.x{i} = gait(1 + 2*n).states.x(1,:); 
       Body.y{i} = gait(1 + 2*n).states.x(2,:);   
       Body.z{i} = gait(1 + 2*n).states.x(3,:);
     Body.yaw{i} = gait(1 + 2*n).states.x(4,:).*(180/pi); 
   Body.pitch{i} = gait(1 + 2*n).states.x(5,:).*(180/pi); 
    Body.roll{i} = gait(1 + 2*n).states.x(6,:).*(180/pi); 
      Body.dx{i} = gait(1 + 2*n).states.dx(1,:); 
      Body.dy{i} = gait(1 + 2*n).states.dx(2,:);   
      Body.dz{i} = gait(1 + 2*n).states.dx(3,:);
    Body.dyaw{i} = gait(1 + 2*n).states.dx(4,:).*(180/pi); 
  Body.dpitch{i} = gait(1 + 2*n).states.dx(5,:).*(180/pi); 
   Body.droll{i} = gait(1 + 2*n).states.dx(6,:).*(180/pi); 
%----------------------Joints Torques-------------------------------%
      HipT.FR{i} = gait(1 + 2*n).inputs.u(1,:);   
      HipT.FL{i} = gait(1 + 2*n).inputs.u(4,:);
      HipT.RR{i} = gait(1 + 2*n).inputs.u(7,:);
      HipT.RL{i} = gait(1 + 2*n).inputs.u(10,:);  
    ThighT.FR{i} = gait(1 + 2*n).inputs.u(2,:);   
    ThighT.FL{i} = gait(1 + 2*n).inputs.u(5,:);
    ThighT.RR{i} = gait(1 + 2*n).inputs.u(8,:);
    ThighT.RL{i} = gait(1 + 2*n).inputs.u(11,:);
     CalfT.FR{i} = gait(1 + 2*n).inputs.u(3,:);   
     CalfT.FL{i} = gait(1 + 2*n).inputs.u(6,:);
     CalfT.RR{i} = gait(1 + 2*n).inputs.u(9,:);
     CalfT.RL{i} = gait(1 + 2*n).inputs.u(12,:); 
%-----------------------Joints States-------------------------------%
      Hip.FR{i} = gait(1 + 2*n).states.x(7,:).*(180/pi);   
      Hip.FL{i} = gait(1 + 2*n).states.x(10,:).*(180/pi);
      Hip.RR{i} = gait(1 + 2*n).states.x(13,:).*(180/pi);
      Hip.RL{i} = gait(1 + 2*n).states.x(16,:).*(180/pi);
    Thigh.FR{i} = gait(1 + 2*n).states.x(8,:).*(180/pi);   
    Thigh.FL{i} = gait(1 + 2*n).states.x(11,:).*(180/pi);
    Thigh.RR{i} = gait(1 + 2*n).states.x(14,:).*(180/pi);
    Thigh.RL{i} = gait(1 + 2*n).states.x(17,:).*(180/pi); 
     Calf.FR{i} = gait(1 + 2*n).states.x(9,:).*(180/pi);   
     Calf.FL{i} = gait(1 + 2*n).states.x(12,:).*(180/pi);
     Calf.RR{i} = gait(1 + 2*n).states.x(15,:).*(180/pi);
     Calf.RL{i} = gait(1 + 2*n).states.x(18,:).*(180/pi);    
     dHip.FR{i} = gait(1 + 2*n).states.dx(7,:).*(180/pi);   
     dHip.FL{i} = gait(1 + 2*n).states.dx(10,:).*(180/pi);
     dHip.RR{i} = gait(1 + 2*n).states.dx(13,:).*(180/pi);
     dHip.RL{i} = gait(1 + 2*n).states.dx(16,:).*(180/pi);
   dThigh.FR{i} = gait(1 + 2*n).states.dx(8,:).*(180/pi);   
   dThigh.FL{i} = gait(1 + 2*n).states.dx(11,:).*(180/pi);
   dThigh.RR{i} = gait(1 + 2*n).states.dx(14,:).*(180/pi);
   dThigh.RL{i} = gait(1 + 2*n).states.dx(17,:).*(180/pi); 
    dCalf.FR{i} = gait(1 + 2*n).states.dx(9,:).*(180/pi);   
    dCalf.FL{i} = gait(1 + 2*n).states.dx(12,:).*(180/pi);
    dCalf.RR{i} = gait(1 + 2*n).states.dx(15,:).*(180/pi);
    dCalf.RL{i} = gait(1 + 2*n).states.dx(18,:).*(180/pi);    
%-----------------------Leg Angle--------------------------------------------------%
   LegAngle.FR{i} = gait(1 + 2*n).states.x(8,:) + gait(1 + 2*n).states.x(9,:)/2;
   LegAngle.FL{i} = gait(1 + 2*n).states.x(11,:) + gait(1 + 2*n).states.x(12,:)/2;
   LegAngle.RR{i} = gait(1 + 2*n).states.x(14,:) + gait(1 + 2*n).states.x(15,:)/2;
   LegAngle.RL{i} = gait(1 + 2*n).states.x(17,:) + gait(1 + 2*n).states.x(18,:)/2;
%-----------------------Leg Length---------------------------------------------------------------%
    LegLength.FR{i} = sqrt((0.2)^2 + (0.2)^2 - 2*0.2*0.2*cos(pi - gait(1 + 2*n).states.x(9,:)));
    LegLength.FL{i} = sqrt((0.2)^2 + (0.2)^2 - 2*0.2*0.2*cos(pi - gait(1 + 2*n).states.x(12,:)));
    LegLength.RR{i} = sqrt((0.2)^2 + (0.2)^2 - 2*0.2*0.2*cos(pi - gait(1 + 2*n).states.x(15,:)));
    LegLength.RL{i} = sqrt((0.2)^2 + (0.2)^2 - 2*0.2*0.2*cos(pi - gait(1 + 2*n).states.x(18,:)));
              n = n + 1;    
end 

switch what
    
    case 'BodyPositions'  
      f = figure;
      f.Units = 'inches';
      f.Position = [0.5 0.5 25.75 9.75];
%      f.ToolBar = 'none'; 
%      f.MenuBar = 'none';
      f.Color = 'w';
      t = tiledlayout(2,3);
      % ---Body X--- %
      nexttile
      plot(Time, cell2mat([Body.x]),'r')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on      
      end 
      grid on
      title(['Body X Position, V = ',num2str(v),' m/s']) 
      xlabel(' Time   (s) ')
      ylabel(' x   (m) ')
      % ---Body Y--- %
      nexttile
      plot(Time, cell2mat([Body.y]),'b')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on        
      end 
      grid on
      title(['Body Y Position, V = ',num2str(v),' m/s']) 
      xlabel(' Time   (s) ')
      ylabel(' y   (m) ')
      % ---Body Z--- %
      nexttile
      plot(Time, cell2mat([Body.z]),'k')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on      
      end 
      grid on
      title(['Body Z Position, V = ',num2str(v),' m/s']) 
      xlabel(' Time   (s) ')
      ylabel(' z   (m) ')
      % ---Body Roll--- %
      nexttile
      plot(Time, cell2mat([Body.roll]),'r')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on       
      end 
      grid on
      title(['Body Roll Angle, V = ',num2str(v),' m/s']) 
      xlabel(' Time   (s) ')
      ylabel(' roll   (deg) ')
      % ---Body Pitch--- %
      nexttile
      plot(Time, cell2mat([Body.pitch]),'b')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on       
      end 
      grid on
      title(['Body Pitch Angle, V = ',num2str(v),' m/s'])  
      xlabel(' Time   (s) ')
      ylabel(' pitch  (deg) ')
      % ---Body Yaw--- %
      nexttile
      plot(Time, cell2mat([Body.yaw]),'k')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on      
      end 
      grid on
      title(['Body Yaw Angle, V = ',num2str(v),' m/s'])  
      xlabel(' Time   (s) ')
      ylabel(' yaw    (deg)')
      
      t.Padding = 'compact';
      t.TileSpacing = 'compact';
      
%-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------% 
    case 'BodyVelocities'  
      f = figure;
      f.Units = 'inches';
      f.Position = [0.5 0.5 25.75 9.75];
%      f.ToolBar = 'none'; 
%      f.MenuBar = 'none';
      f.Color = 'w';
      t = tiledlayout(2,3);
      
      % ---Body dX--- %
      nexttile
      plot(Time, cell2mat([Body.dx]),'r')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on      
      end 
      grid on
      title(['Body X Velocity, V = ',num2str(v),' m/s'])  
      xlabel(' Time   (s) ')
      ylabel(' dx   (m/s) ')
      % ---Body dY--- %
      nexttile
      plot(Time, cell2mat([Body.dy]),'b')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on        
      end 
      grid on
      title(['Body Y Velocity, V = ',num2str(v),' m/s'])  
      xlabel(' Time   (s) ')
      ylabel(' dy   (m/s) ')
      % ---Body dZ--- %
      nexttile
      plot(Time, cell2mat([Body.dz]),'k')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on      
      end 
      grid on
      title(['Body Z Velocity, V = ',num2str(v),' m/s'])
      xlabel(' Time   (s) ')
      ylabel(' dz   (m/s) ')
      % ---Body dRoll--- %
      nexttile
      plot(Time, cell2mat([Body.droll]),'r')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on       
      end 
      grid on
      title(['Body Roll Velocity, V = ',num2str(v),' m/s'])
      xlabel(' Time   (s) ')
      ylabel(' droll   (deg/s) ')
      % ---Body dPitch--- %
      nexttile
      plot(Time, cell2mat([Body.dpitch]),'b')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on       
      end 
      grid on
      title(['Body Pitch Velocity, V = ',num2str(v),' m/s']) 
      xlabel(' Time   (s) ')
      ylabel(' dpitch  (deg/s) ')
      % ---Body dYaw--- %
      nexttile
      plot(Time, cell2mat([Body.dyaw]),'k')
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.','Color', [1 0.5 1],'linewidth',1);
      hold on      
      end 
      grid on
      title(['Body Yaw Velocity, V = ',num2str(v),' m/s'])
      xlabel(' Time   (s) ')
      ylabel(' dyaw   (deg/s)')
      
      t.Padding = 'compact';
      t.TileSpacing = 'compact';     
%-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------%      
      case 'JointsTorques'
      f = figure;
      f.Units = 'inches';
      f.Position = [0.5 5.7 25.5 4.75];
%       f.ToolBar = 'none'; 
%       f.MenuBar = 'none';
      f.Color = 'w';
      t = tiledlayout(1,3);
      
      % ---Hip Joints Torques--- %
      nexttile
      plot(Time, cell2mat([HipT.FR]),'r', Time, cell2mat([HipT.FL]),'b', Time, cell2mat([HipT.RR]),'.-g', Time, cell2mat([HipT.RL]),'.-m' )
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on      
      end 
      grid on
      title(['Hip Joints Torques (Limits -20 to +20), V = ',num2str(v),' m/s']) 
      xlabel(' Time   (s) ')
      ylabel(' Torque   (N.m) ')
      legend('Front Right','Front Left','Rear Right','Rear Left')
      % ---Thigh Joints Torques--- %
      nexttile
      plot(Time, cell2mat([ThighT.FR]),'r', Time, cell2mat([ThighT.FL]),'b', Time, cell2mat([ThighT.RR]),'.-g', Time, cell2mat([ThighT.RL]),'.-m' )
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on            
      end 
      grid on
      title(['Thigh Joints Torques (Limits -55 to +55), V = ',num2str(v),' m/s']) 
      xlabel('Time   (s) ')
      ylabel('Torque  (N.m) ')
      legend('Front Right','Front Left','Rear Right','Rear Left')
      % ---Calf Joints Torques--- %
      nexttile
      plot(Time, cell2mat([CalfT.FR]),'r', Time, cell2mat([CalfT.FL]),'b', Time, cell2mat([CalfT.RR]),'.-g', Time, cell2mat([CalfT.RL]),'.-m' )
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on           
      end 
      grid on
      title(['Calf Joints Torques (Limits -55 to +55), V = ',num2str(v),' m/s']) 
      xlabel('Time   (s) ')
      ylabel('Torque  (N.m) ')
      legend('Front Right','Front Left','Rear Right','Rear Left')
      
      t.Padding = 'compact';
      t.TileSpacing = 'compact';
%------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------% 
      case 'JointsPositions'
      f = figure;
      f.Units = 'inches';
      f.Position = [0.5 0.3 25.5 4.75];
      %f.ToolBar = 'none'; 
      %f.MenuBar = 'none';
      f.Color = 'w';
      t = tiledlayout(1,3);
      
      % ---Hip Joints Trajectories--- %
      nexttile
      plot(Time, cell2mat([Hip.FR]),'r', Time, cell2mat([Hip.FL]),'b', Time, cell2mat([Hip.RR]),'.-g', Time, cell2mat([Hip.RL]),'.-m' ) 
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on
      end   
      grid on
      title(['Hip Joints Position (Limits -+30), V = ',num2str(v),' m/s']) 
      xlabel( ' Time   (s) ' )
      ylabel( ' Angle   (deg) ' )
      legend('Front Right','Front Left','Rear Right','Rear Left')
      % ---Thigh Joints Trajectories--- %
      nexttile
      plot(Time, cell2mat([Thigh.FR]),'r', Time, cell2mat([Thigh.FL]),'b', Time, cell2mat([Thigh.RR]),'.-g', Time, cell2mat([Thigh.RL]),'.-m' )
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on 
      end 
      grid on
      title(['Thigh Joints Position (Limits -30 to 240), V = ',num2str(v),' m/s']) 
      xlabel( ' Time   (s) ' )
      ylabel( ' Angle   (deg) ' )
      legend('Front Right','Front Left','Rear Right','Rear Left')
      % ---Calf Joints Trajectories--- %
      nexttile
      plot(Time, cell2mat([Calf.FR]),'r', Time, cell2mat([Calf.FL]),'b', Time, cell2mat([Calf.RR]),'.-g', Time, cell2mat([Calf.RL]),'.-m' )
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on   
      end 
      grid on
      title(['Calf Joints Position (Limits -52.5 to -154.5), V = ',num2str(v),' m/s']) 
      xlabel( ' Time   (s) ' )
      ylabel( ' Angle   (deg) ' )
      legend('Front Right','Front Left','Rear Right','Rear Left')
  
      t.Padding = 'compact';
      t.TileSpacing = 'compact'; 
%----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------% 
      case 'LegAngle&Length'  
      f = figure;
      f.Units = 'inches';
      f.Position = [5 5.6 16 4.75];  % [figure bottom left corner distance from the screen left edge, figure bottom left corner distance from the screen bottom edge, length, width]  
      %f.ToolBar = 'none'; 
      %f.MenuBar = 'none';
      f.Color = 'w';
      t = tiledlayout(1,2);

      % ---Leg Angles --- %
      nexttile
      plot(Time, cell2mat([LegAngle.FR])*180/pi,'r', Time, cell2mat([LegAngle.FL])*180/pi,'b', ...
           Time, cell2mat([LegAngle.RR])*180/pi,'.-g', Time, cell2mat([LegAngle.RL])*180/pi,'.-m' )     
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on             
      end 
      grid on
      title(['Leg Angles, V = ',num2str(v),' m/s']) 
      xlabel(' Time   (s) ')
      ylabel(' Angle   (deg) ')
      legend('Front Right','Front Left','Rear Right','Rear Left')
      % ---Legs Lengths --- %
      nexttile
      plot(Time, cell2mat([LegLength.FR]),'r', Time, cell2mat([LegLength.FL]),'b', ...
           Time, cell2mat([LegLength.RR]),'.-g', Time, cell2mat([LegLength.RL]),'.-m' )    
      for i = 1 : number_of_phases-1
      xline(Phase_Time{i}(end),'-k','linewidth',1.3);
      xline((Phase_Time{i}(end) + Phase_Time{i}(1))/2,'-.k','linewidth',1.2);
      xline((Phase_Time{i+1}(end) + Phase_Time{i+1}(1))/2,'-.k','linewidth',1.2);
      hold on        
      end 
      grid on
      title(['Leg Length, V = ',num2str(v),' m/s'])  
      xlabel(' Time   (s) ')
      ylabel(' Length   (m) ')
      legend('Front Right','Front Left','Rear Right','Rear Left')
      
      t.Padding = 'compact';
      t.TileSpacing = 'compact';     
end

      