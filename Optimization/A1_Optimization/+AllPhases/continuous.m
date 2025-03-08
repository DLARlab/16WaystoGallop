function domain = continuous(model, load_path, varargin)
    % construct the triple stance domain of a1_lib
    
    % Parse inputs
    p = inputParser;
    p.addOptional('stanceleg', 'none')
    p.addOptional('guard', 'none')
    p.parse(varargin{:});
    parser_results = p.Results;
    
    % Default domain name
    Name = 'Flight'; %#ok<NASGU>

    % make a copy of the robot model
    domain = copy(model);
    
    % Add contacts
    FR_contact = SysHolonomic.FR_toe_contact(domain);
    FL_contact = SysHolonomic.FL_toe_contact(domain);
    RR_contact = SysHolonomic.RR_toe_contact(domain);
    RL_contact = SysHolonomic.RL_toe_contact(domain);

    
    % Choose swing leg
    switch parser_results.stanceleg
        
        % single stance    
        case 'FR'
            Name = 'Stance_FR';
            domain = addContact(domain, FR_contact.frame, FR_contact.fric_coef, FR_contact.geometry, load_path);        
%             Unilateral Constarints
            domain = addUnilateralConstraint(domain,SysUnilateral.FL_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.RR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.RL_toe_height(domain));
        case 'FL' 
            Name = 'Stance_FL';
            domain = addContact(domain, FL_contact.frame, FL_contact.fric_coef, FL_contact.geometry, load_path);
%             Unilateral Constarints
            domain = addUnilateralConstraint(domain,SysUnilateral.FR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.RR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.RL_toe_height(domain));
        case 'RR'
            Name = 'Stance_RR';              
            domain = addContact(domain, RR_contact.frame, RR_contact.fric_coef, RR_contact.geometry, load_path);  
%             Unilateral Constarints
            domain = addUnilateralConstraint(domain,SysUnilateral.RL_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.FR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.FL_toe_height(domain));
        case 'RL'
            Name = 'Stance_RL';
            domain = addContact(domain, RL_contact.frame, RL_contact.fric_coef, RL_contact.geometry, load_path);
%             Unilateral Constarints
            domain = addUnilateralConstraint(domain,SysUnilateral.RR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.FR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.FL_toe_height(domain));
            
        % double stance    
        case 'FRFL'
            Name = 'Stance_FRFL';
            domain = addContact(domain, FR_contact.frame, FR_contact.fric_coef, FR_contact.geometry, load_path);        
            domain = addContact(domain, FL_contact.frame, FL_contact.fric_coef, FL_contact.geometry, load_path);
%             Unilateral Constarints
            domain = addUnilateralConstraint(domain,SysUnilateral.RR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.RL_toe_height(domain));
             
        case 'FRRR'
            Name = 'Stance_FRRR';            
            domain = addContact(domain, FR_contact.frame, FR_contact.fric_coef, FR_contact.geometry, load_path);      
            domain = addContact(domain, RR_contact.frame, RR_contact.fric_coef, RR_contact.geometry, load_path);    
        case 'FRRL'
            Name = 'Stance_FRRL';
            domain = addContact(domain, FR_contact.frame, FR_contact.fric_coef, FR_contact.geometry, load_path);
            domain = addContact(domain, RL_contact.frame, RL_contact.fric_coef, RL_contact.geometry, load_path);           
        case 'FLRR' 
            Name = 'Stance_FLRR';
            domain = addContact(domain, FL_contact.frame, FL_contact.fric_coef, FL_contact.geometry, load_path);  
            domain = addContact(domain, RR_contact.frame, RR_contact.fric_coef, RR_contact.geometry, load_path);        
        case 'FLRL'
            Name = 'Stance_FLRL';
            domain = addContact(domain, FL_contact.frame, FL_contact.fric_coef, FL_contact.geometry, load_path);   
            domain = addContact(domain, RL_contact.frame, RL_contact.fric_coef, RL_contact.geometry, load_path);            
        case 'RRRL'
            Name = 'Stance_RRRL';
            domain = addContact(domain, RR_contact.frame, RR_contact.fric_coef, RR_contact.geometry, load_path);       
            domain = addContact(domain, RL_contact.frame, RL_contact.fric_coef, RL_contact.geometry, load_path);
%             % Unilateral Constarints
            domain = addUnilateralConstraint(domain,SysUnilateral.FR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.FL_toe_height(domain));
            
        case 'FRFLRRRL'
            Name = 'Stance_FRFLRRRL';
            domain = addContact(domain, FR_contact.frame, FR_contact.fric_coef, FR_contact.geometry, load_path);        
            domain = addContact(domain, FL_contact.frame, FL_contact.fric_coef, FL_contact.geometry, load_path);            
            domain = addContact(domain, RR_contact.frame, RR_contact.fric_coef, RR_contact.geometry, load_path);       
            domain = addContact(domain, RL_contact.frame, RL_contact.fric_coef, RL_contact.geometry, load_path); 
            
        % flight    
        case 'none'
            Name = 'Flight';
            % Unilateral Constarints
            domain = addUnilateralConstraint(domain,SysUnilateral.FL_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.FR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.RR_toe_height(domain));
            domain = addUnilateralConstraint(domain,SysUnilateral.RL_toe_height(domain));
        
        otherwise
            error('Unknown phase')            
            
    end    
      
    
    switch parser_results.guard
        
        case 'none'
            % Do not add any events
            
        % lift-off events
        % single lift-off
        case 'FRlo'
            % Add event - ground reaction force of front right toe
            domain = addEvent(domain,SysUnilateral.FR_toe_ground_reaction_force(domain));
        case 'FLlo'
            % Add event - ground reaction force of front left toe
            domain = addEvent(domain,SysUnilateral.FL_toe_ground_reaction_force(domain));    
        case 'RRlo'
            % Add event - ground reaction force of rear right toe
            domain = addEvent(domain,SysUnilateral.RR_toe_ground_reaction_force(domain));            
        case 'RLlo'
            % Add event - ground reaction force of rear left toe
            domain = addEvent(domain,SysUnilateral.RL_toe_ground_reaction_force(domain)); 
  
        % double lift-off    
        case 'FRFLlo'
            % Add event - ground reaction force of both front toes
            % set guard (impact for now, we remove all impact constraints though)
            domain = addEvent(domain,SysUnilateral.FR_toe_ground_reaction_force(domain));      
            domain = addEvent(domain,SysUnilateral.FL_toe_ground_reaction_force(domain)); 
        case 'RRRLlo'
            % set guard (impact for now, we remove all impact constraints though)
            % Add event - ground reaction force of both hind toes
            domain = addEvent(domain,SysUnilateral.RR_toe_ground_reaction_force(domain)); 
            domain = addEvent(domain,SysUnilateral.RL_toe_ground_reaction_force(domain)); 
            
       % quadrupal lift-off
       case 'FRFLRRRLlo'
            % set guard (impact for now, we remove all impact constraints though)
            % Add event - ground reaction force of all four toes
            domain = addEvent(domain,SysUnilateral.FR_toe_ground_reaction_force(domain));      
            domain = addEvent(domain,SysUnilateral.FL_toe_ground_reaction_force(domain)); 
            domain = addEvent(domain,SysUnilateral.RR_toe_ground_reaction_force(domain)); 
            domain = addEvent(domain,SysUnilateral.RL_toe_ground_reaction_force(domain)); 
            
            
        % touch-down events
        % single touch-down
        case 'FRtd'
            % Add event - height of non-stance foot FR toe
            domain = addEvent(domain,SysUnilateral.FR_toe_height(domain));
        case 'FLtd'
            % Add event - height of non-stance foot FL toe
            domain = addEvent(domain,SysUnilateral.FL_toe_height(domain));    
        case 'RRtd'
            % Add event - height of non-stance foot RR toe
            domain = addEvent(domain,SysUnilateral.RR_toe_height(domain));            
        case 'RLtd'
            % Add event - height of non-stance foot RL toe
            domain = addEvent(domain,SysUnilateral.RL_toe_height(domain)); 
     
        % double touch-down    
        case 'FRFLtd'
            % Add event - height of non-stance foot both front toes
            domain = addEvent(domain,SysUnilateral.FR_toe_height(domain));
            domain = addEvent(domain,SysUnilateral.FL_toe_height(domain));  
        case 'RRRLtd'
            % Add event - height of non-stance foot both hind toes
            domain = addEvent(domain,SysUnilateral.RR_toe_height(domain));
            domain = addEvent(domain,SysUnilateral.RL_toe_height(domain));  
            
        % four touch-down      
        case 'FRFLRRRLtd'    
            % Add event - height of non-stance foot both front toes
            domain = addEvent(domain,SysUnilateral.FR_toe_height(domain));
            domain = addEvent(domain,SysUnilateral.FL_toe_height(domain));  
            domain = addEvent(domain,SysUnilateral.RR_toe_height(domain));
            domain = addEvent(domain,SysUnilateral.RL_toe_height(domain)); 
 
       % quadrupal lift-off & touchdown
        case 'FRRLlo_FLRRtd'
            domain = addEvent(domain,SysUnilateral.FR_toe_ground_reaction_force(domain));      
            domain = addEvent(domain,SysUnilateral.RL_toe_ground_reaction_force(domain)); 
            domain = addEvent(domain,SysUnilateral.FL_toe_height(domain));
            domain = addEvent(domain,SysUnilateral.RR_toe_height(domain));    
            
        case 'FLRRlo_FRRLtd'
            domain = addEvent(domain,SysUnilateral.FL_toe_ground_reaction_force(domain));      
            domain = addEvent(domain,SysUnilateral.RR_toe_ground_reaction_force(domain)); 
            domain = addEvent(domain,SysUnilateral.FR_toe_height(domain));
            domain = addEvent(domain,SysUnilateral.RL_toe_height(domain)); 
            
            
            
        otherwise
            error('Unknown event function')
    end

    % Virtual constraints
    domain = addVirtualConstraint(domain,SysVirtual.actuated_joints(domain, load_path));
    
        
    % Set the name of the new copy
    domain.setName(Name);
   
end