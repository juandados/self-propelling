function u = goOnHighway(obj, veh, hw, target)
% goOnHighway(obj, veh, hw, target)
% method of TFM class

if ~strcmp(veh.q, 'Free') && ~strcmp(veh.q, 'Leader')
  error('Vehicle must be ''Free'' or ''Leader''!')
end

veh.tfm_status = 'busy';

% Control
u = getToPose(obj, veh, target, hw.getHeading);

% If vehicle has reached the right pose
if isempty(u)
  veh.tfm_status = 'idle';
  
  if strcmp(veh.q, 'Leader')
    veh.p.hw = hw;
  else
    veh.q = 'Leader';
    veh.p = Platoon(veh, hw, obj);
  end
end
end