function [sys,domains,guards] = load_behavior(robot, load_path, varargin)   % load gait behavior of A1 %

% (Domains = Phases)  e.g. Flight Phase, Stance Phase , (Guards = Triger Events)  e.g. Touchdown, Lift-off %
% FR = Front Right Leg, FL = Front Left Leg, RR = Rear Right Leg, RL = Rear Left Leg %  

% Parse inputs
p = inputParser;
p.addOptional('type', 'bounding_3')
p.parse(varargin{:});
parser_results = p.Results;   
    
% Choose behavior type %
    switch parser_results.type
%----------------------------------------------------------------------------------------------------------------------------------------------------%
        case 'Pronking'  % stance_FRFLRRRL -> flight %
            
            % --Define domains-- %
            stance_FRFLRRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFLRRRL', 'guard', 'FRFLRRRLlo');           
                     flight = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'FRFLRRRLtd'); % 'FRFLRRRLtd' % RRRLtd
            
            % --Define guards-- %
                FRFLRRRL_td = AllPhases.discrete(robot, load_path, 'leg', 'FRFLRRRL', 'event', 'td', 'next_domain', 'FRFLRRRL');
                FRFLRRRL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FRFLRRRL', 'event', 'lo', 'next_domain', 'flight');
 
            domains = [stance_FRFLRRRL,  flight]; 
             guards = [FRFLRRRL_lo, FRFLRRRL_td]; 
            
            % --Define hybrid system-- %
            srcs = {'stance_FRFLRRRL','flight'};                                    % (srcs = Source)  %
            tars = circshift(srcs,-1); % circularly left shift by 1                 % (tars = Targets) %
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FRFLRRRL, flight});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FRFLRRRL_lo,FRFLRRRL_td}); 
                
%----------------------------------------------------------------------------------------------------------------------------------------------------%
        case 'Trotting'
            
            % --Define domains-- %
            stance_FRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRRL', 'guard', 'FRRLlo_FLRRtd');           
            stance_FLRR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FLRR', 'guard', 'FLRRlo_FRRLtd'); 
            
            % --Define guards-- %
            FRRLlo_FLRRtd = AllPhases.discrete(robot, load_path, 'leg', 'FRRL&FLRR', 'event', 'lotd', 'next_domain', 'FLRR');
            FLRRlo_FRRLtd = AllPhases.discrete(robot, load_path, 'leg', 'FLRR&FRRL', 'event', 'lotd', 'next_domain', 'FRRL');
 
            domains = [  stance_FRRL,   stance_FLRR]; 
             guards = [FRRLlo_FLRRtd, FLRRlo_FRRLtd]; 
            
            % --Define hybrid system-- %
            srcs = {'stance_FRRL','stance_FLRR'};                                    % (srcs = Source)  %
            tars = circshift(srcs,-1); % circularly left shift by 1                 % (tars = Targets) %
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FRRL, stance_FLRR});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FRRLlo_FLRRtd,FLRRlo_FRRLtd}); 
                    
%----------------------------------------------------------------------------------------------------------------------------------------------------%
         case 'BoundingB2' % stance_FRFL -> flight -> stance_RRRL -> flight %
   
            % --Define domains-- % 
            stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRFLlo');          
            stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RRRLlo');     
            flight_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'RRRLtd'); 
            flight_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'FRFLtd'); 
            
            % --Define guards-- %
            RRRL_td = AllPhases.discrete(robot, load_path, 'leg', 'RRRL', 'event', 'td', 'next_domain', 'RRRL');
            FRFL_td = AllPhases.discrete(robot, load_path, 'leg', 'FRFL', 'event', 'td', 'next_domain', 'FRFL');
            RRRL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RRRL', 'event', 'lo', 'next_domain', 'flight');
            FRFL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FRFL', 'event', 'lo', 'next_domain', 'flight');
 
            domains = [stance_FRFL, flight_RRRL, stance_RRRL, flight_FRFL];
            guards = [FRFL_lo, RRRL_td, RRRL_lo, FRFL_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_FRFL','flight_RRRL','stance_RRRL','flight_FRFL'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FRFL, flight_RRRL, stance_RRRL, flight_FRFL});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FRFL_lo, RRRL_td, RRRL_lo, FRFL_td});    
            
%----------------------------------------------------------------------------------------------------------------------------------------------------%   
        case 'BoundingBE'  % stance_FRFL -> stance_FRFLRRRL -> stance_RRRL -> flight %
                
            % --Define domains-- % 
                stance_FRFL = AllPhases.continuous(robot, load_path,'stanceleg','FRFL','guard','RRRLtd');   
            stance_FRFLRRRL = AllPhases.continuous(robot, load_path,'stanceleg','FRFLRRRL','guard','FRFLlo'); 
                stance_RRRL = AllPhases.continuous(robot, load_path,'stanceleg','RRRL','guard','RRRLlo');  
                flight_FRFL = AllPhases.continuous(robot, load_path,'stanceleg','none','guard','FRFLtd');
      
            % --Define guards-- %
                FRFL_td = AllPhases.discrete(robot, load_path,'leg','FRFL','event','td','next_domain','FRFL');
                RRRL_td = AllPhases.discrete(robot, load_path,'leg','RRRL','event','td','next_domain','FRFLRRRL');
                FRFL_lo = AllPhases.discrete(robot, load_path,'leg','FRFL','event','lo','next_domain','RRRL');
                RRRL_lo = AllPhases.discrete(robot, load_path,'leg','RRRL','event','lo','next_domain','flight');
 
            domains = [stance_FRFL, stance_FRFLRRRL, stance_RRRL, flight_FRFL];
             guards = [RRRL_td, FRFL_lo, RRRL_lo, FRFL_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_FRFL', 'stance_FRFLRRRL', 'stance_RRRL', 'flight_FRFL'};   
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FRFL, stance_FRFLRRRL, stance_RRRL, flight_FRFL});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {RRRL_td, FRFL_lo, RRRL_lo, FRFL_td}); 

%----------------------------------------------------------------------------------------------------------------------------------------------------%
         case 'BoundingBG'  % stance_RRRL -> stance_FRFLRRRL -> stance_FRFL -> flight %
                
            % --Define domains-- % 
                stance_RRRL = AllPhases.continuous(robot, load_path,'stanceleg','RRRL','guard','FRFLtd');  
            stance_FRFLRRRL = AllPhases.continuous(robot, load_path,'stanceleg','FRFLRRRL','guard','RRRLlo'); 
                stance_FRFL = AllPhases.continuous(robot, load_path,'stanceleg','FRFL','guard','FRFLlo');   
                flight_RRRL = AllPhases.continuous(robot, load_path,'stanceleg','none','guard','RRRLtd');
      
            % --Define guards-- %
                RRRL_td = AllPhases.discrete(robot, load_path,'leg','RRRL','event','td','next_domain','RRRL');
                FRFL_td = AllPhases.discrete(robot, load_path,'leg','FRFL','event','td','next_domain','FRFLRRRL');
                RRRL_lo = AllPhases.discrete(robot, load_path,'leg','RRRL','event','lo','next_domain','FRFL');
                FRFL_lo = AllPhases.discrete(robot, load_path,'leg','FRFL','event','lo','next_domain','flight');
 
            domains = [stance_RRRL, stance_FRFLRRRL, stance_FRFL, flight_RRRL];
             guards = [FRFL_td, RRRL_lo, FRFL_lo, RRRL_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_RRRL', 'stance_FRFLRRRL', 'stance_FRFL', 'flight_RRRL'};   
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_RRRL, stance_FRFLRRRL, stance_FRFL, flight_RRRL});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FRFL_td, RRRL_lo, FRFL_lo, RRRL_td}); 
             
%----------------------------------------------------------------------------------------------------------------------------------------------------%
        case 'BoundingB0'  % stance_FRFL -> stance_FRFLRRRL -> stance_RRRL -> stance_FRFLRRRL %
                
            % --Define domains-- % 
                stance_FRFL = AllPhases.continuous(robot, load_path,'stanceleg','FRFL','guard','RRRLtd');   
           stance_FRFLRRRL1 = AllPhases.continuous(robot, load_path,'stanceleg','FRFLRRRL','guard','FRFLlo'); 
                stance_RRRL = AllPhases.continuous(robot, load_path,'stanceleg','RRRL','guard','FRFLtd'); 
           stance_FRFLRRRL2 = AllPhases.continuous(robot, load_path,'stanceleg','FRFLRRRL','guard','RRRLlo'); 
      
            % --Define guards-- %
                FRFL_td = AllPhases.discrete(robot, load_path,'leg','FRFL','event','td','next_domain','FRFLRRRL');
                RRRL_td = AllPhases.discrete(robot, load_path,'leg','RRRL','event','td','next_domain','FRFLRRRL');
                FRFL_lo = AllPhases.discrete(robot, load_path,'leg','FRFL','event','lo','next_domain','RRRL');
                RRRL_lo = AllPhases.discrete(robot, load_path,'leg','RRRL','event','lo','next_domain','FRFL');
 
            domains = [stance_FRFL, stance_FRFLRRRL1, stance_RRRL, stance_FRFLRRRL2];
             guards = [    RRRL_td,          FRFL_lo,     FRFL_td,          RRRL_lo];
            
            % --Define hybrid system-- %
            srcs = {'stance_FRFL', 'stance_FRFLRRRL1', 'stance_RRRL', 'stance_FRFLRRRL2'};   
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FRFL, stance_FRFLRRRL1, stance_RRRL, stance_FRFLRRRL2});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {RRRL_td, FRFL_lo, FRFL_td, RRRL_lo}); 

%----------------------------------------------------------------------------------------------------------------------------------------------------%
         case 'TGallopingG2' 
   
            % --Define domains-- % 
            stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FLtd');          
          stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRlo');  
            stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'FLlo');     
              flight1 = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'RRtd'); 
            stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'RLtd');                
          stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RRlo');   
            stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'RLlo'); 
              flight2 = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'FRtd'); 
            
            % --Define guards-- %
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FRFL');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'FL');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'flight');
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'RR');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'RRRL');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'RL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'flight');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FR');

 
            domains = [stance_FR, stance_FRFL, stance_FL, flight1, stance_RR, stance_RRRL, stance_RL, flight2];
             guards = [    FL_td,       FR_lo,     FL_lo,   RR_td,     RL_td,       RR_lo,     RL_lo,   FR_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_FR', 'stance_FRFL', 'stance_FL', 'flight1', 'stance_RR', 'stance_RRRL', 'stance_RL', 'flight2'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FR, stance_FRFL, stance_FL, flight1, stance_RR, stance_RRRL, stance_RL, flight2});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FL_td, FR_lo, FL_lo, RR_td, RL_td, RR_lo, RL_lo, FR_td});   
%----------------------------------------------------------------------------------------------------------------------------------------------------%
          case 'RGallopingG2' 
   
            % --Define domains-- % 
            stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FLtd');          
          stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRlo');  
            stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'FLlo');     
              flight1 = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'RLtd'); 
            stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'RRtd'); 
          stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RLlo');   
            stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'RRlo');                
              flight2 = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'FRtd'); 
            
            % --Define guards-- %
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FRFL');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'FL');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'flight');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'RL');
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'RRRL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'RR');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'flight');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FR');

 
            domains = [stance_FR, stance_FRFL, stance_FL, flight1, stance_RL, stance_RRRL, stance_RR, flight2];
             guards = [    FL_td,       FR_lo,     FL_lo,   RL_td,     RR_td,       RL_lo,     RR_lo,   FR_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_FR', 'stance_FRFL', 'stance_FL', 'flight1', 'stance_RL', 'stance_RRRL', 'stance_RR', 'flight2'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FR, stance_FRFL, stance_FL, flight1, stance_RL, stance_RRRL, stance_RR, flight2});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FL_td, FR_lo, FL_lo, RL_td, RR_td, RL_lo, RR_lo, FR_td});   
%----------------------------------------------------------------------------------------------------------------------------------------------------%
           case 'RGallopingGG' 
   
            % --Define domains-- % 
          stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'RRtd'); 
        stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RLlo');   
          stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'FRtd'); 
        stance_FRRR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRRR', 'guard', 'RRlo');  
          stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FLtd');          
        stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRlo');  
          stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'FLlo');     
             flight = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'RLtd'); 
            
            % --Define guards-- %
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'RRRL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'RR');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FRRR');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'FR');
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FRFL');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'FL');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'flight');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'RL');
              
 
            domains = [ stance_RL, stance_RRRL, stance_RR, stance_FRRR, stance_FR, stance_FRFL, stance_FL, flight];
             guards = [     RR_td,       RL_lo,     FR_td,       RR_lo,     FL_td,       FR_lo,     FL_lo,  RL_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_RL', 'stance_RRRL', 'stance_RR', 'stance_FRRR', 'stance_FR', 'stance_FRFL', 'stance_FL', 'flight'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_RL, stance_RRRL, stance_RR, stance_FRRR, stance_FR, stance_FRFL, stance_FL, flight});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {RR_td, RL_lo, FR_td, RR_lo, FL_td, FR_lo, FL_lo, RL_td});
%----------------------------------------------------------------------------------------------------------------------------------------------------%
           case 'TGallopingGG' 
   
            % --Define domains-- % 
          stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'RRtd'); 
        stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RLlo');   
          stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'FLtd'); 
        stance_FLRR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FLRR', 'guard', 'RRlo'); 
          stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'FRtd'); 
        stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FLlo');  
          stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FRlo');          
             flight = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'RLtd'); 
            
            % --Define guards-- %
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'RRRL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'RR');
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FLRR');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'FL');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FRFL');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'FR');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'flight');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'RL');
              
 
            domains = [ stance_RL, stance_RRRL, stance_RR, stance_FLRR, stance_FL, stance_FRFL, stance_FR, flight];
             guards = [     RR_td,       RL_lo,     FL_td,       RR_lo,     FR_td,       FL_lo,     FR_lo,  RL_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_RL', 'stance_RRRL', 'stance_RR', 'stance_FLRR', 'stance_FL', 'stance_FRFL', 'stance_FR', 'flight'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', { stance_RL, stance_RRRL, stance_RR, stance_FLRR, stance_FL, stance_FRFL, stance_FR, flight});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {RR_td, RL_lo, FL_td, RR_lo, FR_td, FL_lo, FR_lo, RL_td});
%----------------------------------------------------------------------------------------------------------------------------------------------------%
           case 'RGallopingGE' 
   
            % --Define domains-- % 
          stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FLtd');          
        stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRlo');  
          stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'RLtd');     
        stance_FLRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FLRL', 'guard', 'FLlo'); 
          stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'RRtd');                
        stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RLlo');   
          stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'RRlo'); 
             flight = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'FRtd'); 
            
            % --Define guards-- %
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FRFL');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'FL');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'FLRL');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'RL');
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'RRRL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'RR');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'flight');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FR');

              
            domains = [stance_FR, stance_FRFL, stance_FL, stance_FLRL, stance_RL, stance_RRRL, stance_RR, flight];
             guards = [    FL_td,       FR_lo,     RL_td,       FL_lo,     RR_td,     RL_lo,     RR_lo,   FR_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_FR', 'stance_FRFL', 'stance_FL', 'stance_FLRL', 'stance_RL', 'stance_RRRL', 'stance_RR', 'flight'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FR, stance_FRFL, stance_FL, stance_FLRL, stance_RL, stance_RRRL, stance_RR, flight});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FL_td, FR_lo, RL_td, FL_lo, RR_td, RL_lo, RR_lo, FR_td});   
%----------------------------------------------------------------------------------------------------------------------------------------------------%
           case 'TGallopingGE' 
   
            % --Define domains-- % 
          stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FLtd');          
        stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRlo');  
          stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'RRtd');     
        stance_FLRR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FLRR', 'guard', 'FLlo'); 
          stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'RLtd');                
        stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RRlo');   
          stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'RLlo'); 
             flight = AllPhases.continuous(robot, load_path, 'stanceleg', 'none', 'guard', 'FRtd'); 
            
            % --Define guards-- %
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FRFL');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'FL');
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'FLRR');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'RR');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'RRRL');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'RL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'flight');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FR');

              
            domains = [stance_FR, stance_FRFL, stance_FL, stance_FLRR, stance_RR, stance_RRRL, stance_RL, flight];
             guards = [    FL_td,       FR_lo,     RR_td,       FL_lo,     RL_td,     RR_lo,     RL_lo,   FR_td];
            
            % --Define hybrid system-- %
            srcs = {'stance_FR', 'stance_FRFL', 'stance_FL', 'stance_FLRR', 'stance_RR', 'stance_RRRL', 'stance_RL', 'flight'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FR, stance_FRFL, stance_FL, stance_FLRR, stance_RR, stance_RRRL, stance_RL, flight});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FL_td, FR_lo, RR_td, FL_lo, RL_td, RR_lo, RL_lo, FR_td});   
%----------------------------------------------------------------------------------------------------------------------------------------------------%
           case 'RGallopingG0' 
   
            % --Define domains-- % 
          stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FLtd');          
        stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRlo');  
          stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'RLtd');     
        stance_FLRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FLRL', 'guard', 'FLlo'); 
          stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'RRtd');                
        stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RLlo');   
          stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'FRtd'); 
        stance_FRRR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRRR', 'guard', 'RRlo'); 
            
            % --Define guards-- %
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FRFL');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'FL');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'FLRL');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'RL');
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'RRRL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'RR');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FRRR');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'FR');
              
            domains = [stance_FR, stance_FRFL, stance_FL, stance_FLRL, stance_RL, stance_RRRL, stance_RR, stance_FRRR];
             guards = [    FL_td,       FR_lo,     RL_td,       FL_lo,     RR_td,       RL_lo,     FR_td,       RR_lo];
            
            % --Define hybrid system-- %
            srcs = {'stance_FR', 'stance_FRFL', 'stance_FL', 'stance_FLRL', 'stance_RL', 'stance_RRRL', 'stance_RR', 'stance_FRRR'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FR, stance_FRFL, stance_FL, stance_FLRL, stance_RL, stance_RRRL, stance_RR, stance_FRRR});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FL_td, FR_lo, RL_td, FL_lo, RR_td, RL_lo, FR_td, RR_lo});  
             
%----------------------------------------------------------------------------------------------------------------------------------------------------%       
        case 'TGallopingG0' 
   
            % --Define domains-- % 
          stance_FR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FR',   'guard', 'FLtd');          
        stance_FRFL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRFL', 'guard', 'FRlo');  
          stance_FL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FL',   'guard', 'RRtd');     
        stance_FLRR = AllPhases.continuous(robot, load_path, 'stanceleg', 'FLRR', 'guard', 'FLlo'); 
          stance_RR = AllPhases.continuous(robot, load_path, 'stanceleg', 'RR',   'guard', 'RLtd');                
        stance_RRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RRRL', 'guard', 'RRlo');   
          stance_RL = AllPhases.continuous(robot, load_path, 'stanceleg', 'RL',   'guard', 'FRtd'); 
        stance_FRRL = AllPhases.continuous(robot, load_path, 'stanceleg', 'FRRL', 'guard', 'RLlo'); 
            
            % --Define guards-- %
              FL_td = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'td', 'next_domain', 'FRFL');
              FR_lo = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'lo', 'next_domain', 'FL');
              RR_td = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'td', 'next_domain', 'FLRR');
              FL_lo = AllPhases.discrete(robot, load_path, 'leg', 'FL', 'event', 'lo', 'next_domain', 'RR');
              RL_td = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'td', 'next_domain', 'RRRL');
              RR_lo = AllPhases.discrete(robot, load_path, 'leg', 'RR', 'event', 'lo', 'next_domain', 'RL');
              FR_td = AllPhases.discrete(robot, load_path, 'leg', 'FR', 'event', 'td', 'next_domain', 'FRRL');
              RL_lo = AllPhases.discrete(robot, load_path, 'leg', 'RL', 'event', 'lo', 'next_domain', 'FR');

              
            domains = [stance_FR, stance_FRFL, stance_FL, stance_FLRR, stance_RR, stance_RRRL, stance_RL, stance_FRRL];
             guards = [    FL_td,       FR_lo,     RR_td,       FL_lo,     RL_td,       RR_lo,     FR_td,       RL_lo];
            
            % --Define hybrid system-- %
            srcs = {'stance_FR', 'stance_FRFL', 'stance_FL', 'stance_FLRR', 'stance_RR', 'stance_RRRL', 'stance_RL', 'stance_FRRL'};      
            tars = circshift(srcs,-1);                 
             sys = HybridSystem('A1');
             sys = addVertex(sys, srcs, 'Domain', {stance_FR, stance_FRFL, stance_FL, stance_FLRR, stance_RR, stance_RRRL, stance_RL, stance_FRRL});
             sys = addEdge(sys, srcs, tars);
             sys = setEdgeProperties(sys, srcs, tars, 'Guard', {FL_td, FR_lo, RR_td, FL_lo, RL_td, RR_lo, FR_td, RL_lo});   
%----------------------------------------------------------------------------------------------------------------------------------------------------%
        otherwise
              error('Unknown behavior type')
            
    end
    
    
end

