function [gait] = parse(nlp, sol)
    %% parse the optimization results to more readible structure data
    
    [tspan, states, inputs, params] = exportSolution(nlp, sol);

   
    tspan{3} = tspan{1}(end) + (tspan{3} - tspan{3}(1));
    tspan{5} = tspan{3}(end) + (tspan{5} - tspan{5}(1));
    tspan{7} = tspan{5}(end) + (tspan{7} - tspan{7}(1));
    tspan{9} = tspan{7}(end) + (tspan{9} - tspan{9}(1));
   tspan{11} = tspan{9}(end) + (tspan{11} - tspan{11}(1));
   tspan{13} = tspan{11}(end) + (tspan{13} - tspan{13}(1));    
   tspan{15} = tspan{13}(end) + (tspan{15} - tspan{15}(1));       
    
    gait = struct(...
        'tspan', tspan,...
        'states', states,...
        'inputs', inputs,...
        'params', params);


end

