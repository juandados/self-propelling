function regVehicle(obj, vehicle)
% function regVehicle(obj, agents)
%
% Registers a vehicle
%
% Mo Chen 2015-11-03
% Modified: Mo Chen, 2015-11-19

% If adding an empty set, do nothing
if isempty(vehicle)
  return
end

% Check vehicle type
if ~isa(vehicle, 'UAS')
  error('Agent must be a UAS!')
end

% Expand cell array and assign an ID to the agent
obj.aas{length(obj.aas) + 1, 1} = vehicle;
vehicle.ID = length(obj.aas);
end