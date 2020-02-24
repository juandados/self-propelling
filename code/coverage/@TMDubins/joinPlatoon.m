function u = joinPlatoon(obj, veh, platoon)
% u = joinPlatoon(obj, platoon)
% method of TFM class
%
% Tells the vehicle veh how to join the platoon
%
% Inputs: obj     - tfm object
%         veh     - the vehicle joining the platoon
%         platoon - the platoon
% Output: u       - control signal for joining platoon
%
% Mo Chen, 2015-11-22

% if ~strcmp(veh.q, 'Free') && ~strcmp(veh.q, 'Leader')
%   error('Vehicle must be ''Free'' or ''Leader''!')
% end

veh.tfm_status = 'busy';

% Check to see if the vehicle's target platoon is already set to this
% platoon. If not, set this platoon to the target platoon for the vehicle
idx = platoon.findJoiner(veh);
if isempty(idx)
  idx = platoon.addJoiner(veh);
end

% Compute phantom position
pPh_rel = platoon.phantomPosition(obj.ipsd, idx);

% Compute control
u = getToRelpos(obj, veh, platoon.vehicles{1}, pPh_rel);

% After merging, update properties
if isempty(u)
  veh.tfm_status = 'idle';
  platoon.removeJoiner(idx);
  
  % If vehicle is free, we're done
  if strcmp(veh.q, 'Free')
    platoon.insertVehicle(veh, idx);
    u = veh.followPlatoon(obj); % Needs to be after vehicle is a follower
    return
  end
  
  % If vehicle is a leader, insert all followers
  % Keep in mind that there may be empty slots
  for i = 1:length(veh.p.vehicles)
    if ~isempty(veh.p.vehicles{i})
      platoon.insertVehicle(veh.p.vehicles{i}, idx);
      idx = idx + 1;
    end
  end
  u = veh.followPlatoon(obj); % Needs to be after vehicle is a follower
end
end