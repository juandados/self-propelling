function [safe, uSafeOptimal] = checkAASafety(obj)
% function [safe, uSafe] = checkAASafety(obj)
%
% Checks the safety of every active agent with respect to all other active
% agents
%
% Input:   obj   - tfm object
% outputs: safe  - boolean vector; row i specifies whether agent i is safe
%                  with respect to all other agents
%          uSafe - cell vector of safety-preserving controls
%
% Juan Chacon, 2020-jun-03

% uSafe needs to be cell because controls may have different dimensions in
% different agents
safe = ones(length(obj.aas));
uSafeOptimal = cell(length(obj.aas), 1);

% Go through every pair of agents and return safety indicator and control
for i = 1:length(obj.aas)
  uSafeOptimali = cell(1, length(obj.aas));
  % Check safety against all other free vehicles
  if strcmp(obj.aas{i}.q, 'Free')
    for j = 1:length(obj.aas)
      if strcmp(obj.aas{j}.q, 'Free') 
        [safe(i,j), uSafeOptimali{j}] = obj.checkPWSafety(i, j);
      end
    end
  end
  
  % There should not be more than one conflict
  if nnz(~safe(i,:)) > 1
    disp('More than one conflict detected!')
  end
  
  % Update safe-preserving control for agent i
  if nnz(~safe(i,:)) >= 1
      uSafeOptimal{i} = uSafeOptimali{~safe(i,:)};
  end
end

% Overall safety indicator; vehicle i is safe overall only if it is safe
% with respect to all other vehicles against which safety is checked
safe = prod(safe, 2);

end