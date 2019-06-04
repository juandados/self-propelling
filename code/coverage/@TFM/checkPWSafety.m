function [safe, uSafe, safe_val] = checkPWSafety(obj, i, j)
% function checkPWSafety(obj, i, j)
%          (check pairwise safety)
%
% Checks the safety of active agent i with respect to active agent j, and
% gives a safety-preserving input if necessary
%
% Inputs:  obj - tfm object
%          i   - index of agent for whom safety is checked (evader)
%          j   - index of agent against whom safety is checked (pursuer)
% Outputs: safe - boolean specifying whether agent i is safe with respect
%                 to agent j
%          uSafe - safety-preserving input (empty if not needed)
%
% Mo Chen, 2015-11-04
% Modified: Mo Chen, 2015-12-13

% Vehicle is safe with respect to itself
if isempty(obj.aas{j}) || obj.aas{i} == obj.aas{j}
  safe = 1;
  uSafe = [];
  safe_val = 10;
  return
end

switch(class(obj.aas{i}))
  case 'UTMQuad4D'
    %% agent i is a quadrotor
    switch(class(obj.aas{j}))
      case 'UTMQuad4D'
        [safe, uSafe, safe_val] = checkPWSafety_qr_qr(obj.qr_qr_safe_V, ...
          obj.safetyTime, obj.aas{i}, obj.aas{j});
        
      otherwise
        error('Unknown agent type')
        
    end % end inner switch
    
  case 'Plane'
    %% agent i is a Plane
    switch(class(obj.aas{j}))
      case 'Plane'
        [safe, uSafe, safe_val] = checkPWSafety_pl_pl(obj.pl_pl_safe_V, ...
          obj.aas{i}, obj.aas{j});
        
      otherwise
        error('Only plane-plane safety can be checked right now.')
    end
  case 'Plane4'
    %%  agent i is a Plane4
    switch(class(obj.aas{j}))
      case 'Plane4'
        [safe, uSafe, safe_val] = checkPWSafety_pl4_pl4(obj.pl4_pl4_safe_V, ...
          obj.safetyTime, obj.aas{i}, obj.aas{j});
        
      otherwise
        error('Plane4 can only check safety against Plane4 right now.');
    end
  otherwise
    error('Unknown agent type')
end % end outer switch

end % end function

%%
function [safe, uSafe, valuex] = checkPWSafety_qr_qr(qr_qr_safe_V, safetyTime, ...
  evader, pursuer)
% Safety of quadrotor qr1 with respect to quadrotor qr2

% Heading of evader
theta = evader.getHeading;

% Get relative state assuming pursuer faces 0 degrees
base_pos = rotate2D(evader.getPosition - pursuer.getPosition, -theta);
base_vel = rotate2D(evader.getVelocity - pursuer.getVelocity, -theta);
base_x = [base_pos(1); base_vel(1); base_pos(2); base_vel(2)];

% Check if state is within grid bounds for a closer safety check
if any(base_x <= qr_qr_safe_V.g.min) || ...
    any(base_x >= qr_qr_safe_V.g.max)
  safe = 1;
  uSafe = [];
  valuex = 10;
  return
end

% Compute safety value
valuex = eval_u(qr_qr_safe_V.g, qr_qr_safe_V.data, base_x);

% Compute safety preserving control if needed
if valuex > safetyTime
  % Safe; no need to worry about computing controller
  safe = 1;
  uSafe = [];
  return
end

% Not safe, compute safety controller
safe = 0;

% Compute control assuming "pursuer" is facing 0 degrees
base_grad = calculateCostate(qr_qr_safe_V.g, qr_qr_safe_V.grad, base_x);
ux = (base_grad(2)>=0)*evader.uMax + (base_grad(2)<0)*evader.uMin;
uy = (base_grad(4)>=0)*evader.uMax + (base_grad(4)<0)*evader.uMin;
u = [ux; uy];

% Rotate the control to correspond with the actual heading of the
% "pursuer"
uSafe = rotate2D(u, theta);  

end

%%
function [safe, uSafe, valuex] = checkPWSafety_pl_pl(pl_pl_safe_V, pl1, pl2)
% Safety of plane 1 with respect to plane 2
% Notice that even though Plane has a 4D state space, currently we're still
% using a 3D safety value function, assuming a constant velocity

safety_threshold = 0.1;

% Get relative states
xr = pl2.x(1:3) - pl1.x(1:3);

% rotate position vector so it is correct relative to xe heading
xr(1:2) = rotate2D(xr(1:2), -pl1.x(3));

% Wrap angle if needed
if xr(3) >= 2*pi
  xr(3) = xr(3) - 2*pi;
end

if xr(3) < 0
  xr(3) = xr(3) + 2*pi;
end

% Check if state is within grid bounds for a closer safety check
if any(xr <= pl_pl_safe_V.g.min) || ...
    any(xr >= pl_pl_safe_V.g.max)
  safe = 1;
  uSafe = [];
  valuex = 10;
  return
end

% Evaluate safety value function
valuex = eval_u(pl_pl_safe_V.g, pl_pl_safe_V.data, xr);

% Compute safety controller if necessary
if valuex <= safety_threshold
  safe = 1;
  uSafe = [];
else
  safe = 0;
  
  p = calculateCostate(pl_pl_safe_V.g, pl_pl_safe_V.grad, xr);
  if p(1) * xr(2) - p(2) * xr(1) - p(3) >= 0
    uSafe = pl1.wMax;
  else
    uSafe = -pl1.wMax;
  end
end
end
%%
function [safe, uSafe, valuex] = checkPWSafety_pl4_pl4(pl4_pl4_safe_V, safetyTime, ...
  pl4_1, pl4_2)
% Safety of Plane4 pl4_1 with respect to Plane4 pl4_2

% Heading of "pursuer"
theta = pl4_2.getHeading;

% Get relative state assuming pursuer faces 0 degrees
base_pos = rotate2D(pl4_2.getPosition - pl4_1.getPosition, -theta);
base_vel = rotate2D(pl4_2.getVelocity - pl4_1.getVelocity, -theta);
base_x = [base_pos(1); base_vel(1); base_pos(2); base_vel(2)];

% Check if state is within grid bounds for a closer safety check
if any(base_x <= pl4_pl4_safe_V.g.min) || ...
    any(base_x >= pl4_pl4_safe_V.g.max)
  safe = 1;
  uSafe = [];
  valuex = 10;
  return
end

% Compute safety value
valuex = eval_u(pl4_pl4_safe_V.g, pl4_pl4_safe_V.data, base_x);

% Compute safety preserving control if needed
if valuex > safetyTime
  % Safe; no need to worry about computing controller
  safe = 1;
  uSafe = [];
  return
end

% Not safe, compute safety controller
safe = 0;

pl4_1_uMax = 3 / sqrt(2);
pl4_1_uMin = -pl4_1_uMax;
% Compute control assuming "pursuer" is facing 0 degrees
base_grad = calculateCostate(pl4_pl4_safe_V.g, pl4_pl4_safe_V.grad, base_x);
ux = (base_grad(2)>=0)*pl4_1_uMin + (base_grad(2)<0)*pl4_1_uMax;
uy = (base_grad(4)>=0)*pl4_1_uMin + (base_grad(4)<0)*pl4_1_uMax;
u = [ux; uy];

% Rotate the control to correspond with the actual heading of the
% "pursuer"
uSafe = rotate2D(u, theta);  

end

%%
function [safe, uSafe, valuex] = checkPWSafety_qr_qrp(qr, qrp)
% Safety of quadrotor qr with respect to quadrotor platoon qrp
error('Not implemented yet')
end

function [safe, uSafe, valuex] = checkPWSafety_qrp_qr(qrp, qr)
% Safety of quadrotor platoon qrp with respect to quadrotor qr
error('Not implemented yet')
end

function [safe, uSafe, valuex] = checkPWSafety_qrp_qrp(qrp1, qrp2)
% Safety of quadrotor platoon qrp1 with respect to quadrotor platoon qrp2
error('Not implemented yet')
end