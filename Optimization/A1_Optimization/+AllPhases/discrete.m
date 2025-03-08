% Foot touch down (guard)
function guard = discrete(model, load_path, varargin)
    
    % Parse inputs
    p = inputParser;
    p.addOptional('leg', 'FR')
    p.addOptional('event', 'td') % td: touch-down lo: lift-off
    p.addOptional('next_domain', 'FRFL')
    p.addOptional('relabel', false)
    p.parse(varargin{:});
    parser_results = p.Results;
    
    % Default guard name
    Name = parser_results.event;
            
    % Specify next domain
    switch parser_results.next_domain
        case 'FR' 
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FR');         
        case 'FL'
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FL');                 
        case 'RR' 
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'RR');        
        case 'RL' 
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'RL');
        case 'FRFL'
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FRFL');   
        case 'FRRL'
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FRRL');                 
        case 'FRRR' 
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FRRR');     
        case 'FLRR'    
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FLRR');                  
        case 'FLRL'    
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FLRL');    
        case 'RRRL' 
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'RRRL'); 
        case 'FRFLRRRL' 
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'FRFLRRRL');      
        case 'flight'  
            domain   = AllPhases.continuous(model, load_path, 'stanceleg', 'none');
        otherwise
            error('Unknown domain')
    end
    
    switch parser_results.event
        
        case 'td'
            % Choose based on impact leg
            switch parser_results.leg
                case 'FR'
                    % Default domain name
                    Name = ['FR_', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,'FRtd');                        
                case 'FL'
                    % Default domain name
                    Name = ['FL_', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,'FLtd');
                case 'RR'
                    % Default domain name
                    Name = ['RR_', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,'RRtd');
                case 'RL'
                    % Default domain name
                    Name = ['RL_', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,'RLtd');                    
                case 'FRFL'
                    % Default domain name
                    Name = ['FRFL', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,{'FRtd';'FLtd'}); 
                    % TODO (ZG): should be both toe heights
                case 'RRRL'
                    % Default domain name
                    Name = ['RRRL', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,{'RRtd';'RLtd'});              
                    % TODO (ZG): should be both toe heights
                case 'FRFLRRRL'
                    % Default domain name
                    Name = ['FRFLRRRL', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,{'FRtd';'FLtd';'RRtd';'RLtd'});  % RRtd ---- {'FRtd';'FLtd';'RRtd';'RLtd'}
                    %guard = RigidImpact(Name,domain,'FRFLRRRLtd');
                    % TODO (ZG): should be both toe heights   
                    % The name of the Rigid impact should match the one for
                    % the discrete state
                    
                otherwise
                    error('Unknown leg.')
            end
            
        case 'lo'   
                % Choose based on lifted leg
            switch parser_results.leg
                case 'FR'
                    Name = ['FR_',Name];
                    % set guard (impact for now, we remove all impact constraints though)
                    guard = RigidImpact(Name, domain, 'FRlo');    
                case 'FL'
                    Name = ['FL_',Name];
                    % set guard (impact for now, we remove all impact constraints though)
                    guard = RigidImpact(Name, domain, 'FLlo');
                case 'RL'
                    Name = ['RL_',Name];
                    % set guard (impact for now, we remove all impact constraints though)
                    guard = RigidImpact(Name, domain, 'RLlo');                    
                case 'RR'
                    Name = ['RR_',Name];
                    % set guard (impact for now, we remove all impact constraints though)
                    guard = RigidImpact(Name, domain, 'RRlo');                     
                case 'FRFL'
                    Name = ['FRFL',Name];
                    % set guard (impact for now, we remove all impact constraints though)
                    guard = RigidImpact(Name, domain,{'FRlo';'FLlo'}); % Warning: these names have to match the ones in FR_toe_ground_reaction_force
                    % guard = RigidImpact(Name, domain, 'FLlo'); 
                    % TODO (ZG): should be both toe GRFs
                case 'RRRL'
                    Name = ['RRRL',Name]; % this name should be RRRLtd which matches the one in getBound function
                    % set guard (impact for now, we remove all impact constraints though)
                    % input: Name of the guard, next domain, name of the event
                    guard = RigidImpact(Name, domain,{'RRlo';'RLlo'});
                    % Warning: Name of the guard has to match the ones in RR_toe_ground_reaction_force   
                    % and the field name in bound structure
                    % guard = RigidImpact(Name, domain, 'RLlo'); 
                    % TODO (ZG): should be both toe GRFs
                case 'FRFLRRRL'
                    % Default domain name
                    Name = ['FRFLRRRL', Name];
                    % Setup impact
                    guard = RigidImpact(Name,domain,{'FRlo';'FLlo';'RRlo';'RLlo'});  % FRlo ---- {'FRlo';'FLlo';'RRlo';'RLlo'}
                    % TODO (ZG): should be both toe heights       
                    
                    
                otherwise
                    error('Unknown leg')
            end
            
            
            case 'lotd' 
            switch parser_results.leg
                case 'FRRL&FLRR'                    
                     Name = ['FRRLlo_FLRRtd_',Name];    
                     guard = RigidImpact(Name,domain,{'FRlo';'FLtd';'RRtd';'RLlo'});
                case 'FLRR&FRRL'                    
                     Name = ['FLRRlo_FRRLtd_',Name];    
                     guard = RigidImpact(Name,domain,{'FRtd';'FLlo';'RRlo';'RLtd'});
            end 
            
        otherwise
            error('Unknown event.')
    end
    
    if ~strcmp(parser_results.next_domain,'flight') % no holonomic constratins in current flight phase (Cassie has spring and fourbar constraints)
        % Set the impact constraint for stance phase
        guard.addImpactConstraint(struct2array(domain.HolonomicConstraints), load_path);
    end
    
    guard  = configure(guard, load_path); % if not add impact constraint in the previous case, have to manually configure it here to load xMap and dxMap
    
end