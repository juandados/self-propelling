function [safe, uSafe, safe_val] = checkAASafety(obj)
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
% Mo Chen, 2015-11-04
% Modified: Mo Chen, 2015-11-19

% uSafe needs to be cell because controls may have different dimensions in
% different agents
safe = ones(length(obj.aas));
uSafe = cell(length(obj.aas), 1);
safe_val = 10*ones(length(obj.aas));

% Go through every pair of agents and return safety indicator and control
for i = 1:length(obj.aas)
  uSafei = cell(1, length(obj.aas));

  % Free vehicles check safety against 
  %   all other free vehicles
  %   all leaders
  if strcmp(obj.aas{i}.q, 'Free')
    for j = 1:length(obj.aas)
      if strcmp(obj.aas{j}.q, 'Free') || ...
          strcmp(obj.aas{j}.q, 'Leader') 
        [safe(i,j), uSafei{j}, safe_val(i,j)] = obj.checkPWSafety(i, j);
      end
    end
  end

  % Leader vehicles check safety against all other leaders and free
  % vehicles, as well as the vehicle behind him
  if strcmp(obj.aas{i}.q, 'Leader')
    for j = 1:length(obj.aas)
      if obj.aas{j} == obj.aas{i}.BQ || strcmp(obj.aas{j}.q, 'Free') ...
          || strcmp(obj.aas{j}.q, 'Leader')
        [safe(i,j), uSafei{j}, safe_val(i,j)] = obj.checkPWSafety(i, j);
      end
    end
  end

  % Followers check safety against 
  %   Vehicle in the same platoon that is in front
  %   Vehicle in the same platoon that is behind
  %   'Free' vehicles (for CDC 2015 paper)
  if strcmp(obj.aas{i}.q, 'Follower')
    for j = 1:length(obj.aas)
      if strcmp(obj.aas{j}.q, 'Free') || obj.aas{j} == obj.aas{i}.BQ || ...
          obj.aas{j} == obj.aas{i}.FQ
        [safe(i,j), uSafei{j}, safe_val(i,j)] = obj.checkPWSafety(i, j);
      end
    end
  end

  % Assume no safety checks for other vehicles
  
  % There should not be more than one conflict
  if nnz(~safe(i,:)) > 1
    error('More than one conflict detected!')
  end
  
  % Update safe-preserving control for agent i
  if nnz(~safe(i,:)) == 1
    uSafe{i} = uSafei{~safe(i,:)};
  end
end

% Overall safety indicator; vehicle i is safe overall only if it is safe
% with respect to all other vehicles against which safety is checked
safe = prod(safe, 2);

% Minimum safety value
safe_val = min(safe_val, [], 2);
end