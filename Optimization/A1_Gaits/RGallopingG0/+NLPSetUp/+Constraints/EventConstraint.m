function EventConstraint(nlp, src, tar, bounds, varargin)
%     Parse inputs
%     p = inputParser;
%     p.addOptional('leg', 'FRFL')
%     p.parse(varargin{:});
%     parser_results = p.Results;
    
    % First call the class method
    nlp.Plant.rigidImpactConstraint(nlp, src, tar, bounds, varargin{:});
    
    % Don't need time continuity constraint
    removeConstraint(nlp,'tContDomain');
    
    % Remove default constraint and add back selectable one
    removeConstraint(nlp,['xDiscreteMap' nlp.Plant.Name]);
    
    selected = SymVariable('s',[nlp.Plant.numState,1]);
    R = nlp.Plant.R;
    x = nlp.Plant.States.x;
    xn = nlp.Plant.States.xn;
    x_diff = selected.*(R*x-xn);
    x_map = SymFunction(['xDiscreteMap' nlp.Plant.Name],x_diff,{x,xn}, selected);
    
    % Constrain all states
    selected = ones(18,1);
    
    if strcmp(nlp.Name, 'RR_lo')
       % Don't constrain the absolute position and yaw for the last event
       selected(1) = 0; % For not constarining the the first four states variables to be periodic %
    end
    
    addNodeConstraint(nlp, x_map, {'x','xn'}, 'first', 0, 0, 'Linear', selected);
end