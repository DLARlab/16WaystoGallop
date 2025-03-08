function [BezierSymmetryFun] = BezierSymmetry(nlp,Type)
   switch Type 
        case 'Pair' 
            [N,M] = size(nlp.Plant.Params.atime);
         selected = SymVariable('s',[N/2,1]);
               a  = SymVariable('a',[N,M]);        
constraint_matrix = repmat(selected,1,M).*([-a(1,:);-a(7,:);a(2:3,:);a(8:9,:)]-[a(4,:);a(10,:);a(5:6,:);a(11:12,:)]);
       constraint = constraint_matrix(:);
BezierSymmetryFun = SymFunction(['BezierSymmetry',Type],constraint, SymVariable(a(:)), selected);
        %selected = ones(N/2,1);
         
       case 'Side' 
            [N,M] = size(nlp.Plant.Params.atime);
         selected = SymVariable('s',[N/2,1]);
               a  = SymVariable('a',[N,M]);
constraint_matrix = repmat(selected,1,M).*(a(1:6,:)-a(7:12,:));
       constraint = constraint_matrix(:);
BezierSymmetryFun = SymFunction(['BezierSymmetry',Type],constraint, SymVariable(a(:)), selected);
        %selected = ones(N/2,1);
         
       case 'Diagonal' 
            [N,M] = size(nlp.Plant.Params.atime);
         selected = SymVariable('s',[N/2,1]);
               a  = SymVariable('a',[N,M]);
constraint_matrix = repmat(selected,1,M).*([a(1:3,:);a(10:12,:)]-[a(4:6,:);a(7:9,:)]);
       constraint = constraint_matrix(:);
BezierSymmetryFun = SymFunction(['BezierSymmetry',Type],constraint, SymVariable(a(:)), selected);
        %selected = ones(N/2,1);
         
%        case 'All' 
%             [N,M] = size(nlp.Plant.Params.atime);
%          selected = SymVariable('s',[N/2,1]);
%                a  = SymVariable('a',[N,M]);
%          selected = ones(N/2,1);
% constraint_matrix = repmat(selected,1,M).*([-a(1,:);-a(7,:);a(2:3,:);a(8:9,:)]-[a(4,:);a(10,:);a(5:6,:);a(11:12,:)]);
%        constraint = constraint_matrix(:);
% BezierSymmetryFun = SymFunction(['BezierSymmetry_',Gait],constraint, SymVariable(a(:)), selected);
%          selected = ones(N/2,1);
    end
end
