function addActiveAgents(obj, agents)
% function addActiveAgents(obj, agent)
%
% Adds a list of agents to the active list
%
% Mo Chen 2015-11-03

% If adding an empty set, do nothing
if isempty(agents)
  return
end

% Convert a single agent into a list containing the agent if necessary
if ~iscell(agents)
  agents = {agents};
end

% Check vehicle types
for i = 1:length(agents)
  if ~isa(agents{i}, 'Quadrotor') && ~isa(agents{i}, 'Platoon')
    error('For now, agents must be a quadrotor or a platoon!')
  end
end

% Expand cell array
for i = 1:length(agents)
  obj.aas{length(obj.aas) + 1, 1} = agents{i};
  
  % If agent is a vehicle, assign a unique ID
  if isa(agents{i}, 'Vehicle')
    agents{i}.ID = length(obj.aas);
  end
end
end