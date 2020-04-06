function u = getToPose(obj, vehicle, position, heading, debug)
% function u = getToState(obj, vehicle, target_state)
%
% Computes the control that drives the vehicle to some target state
%
% Mo Chen, 2015-11-03

% Make sure target state is valid
if numel(position) ~= 2
  error('Incorrect number of dimensions in target state!')
end

if ~iscolumn(position)
  position = position';
end

if ~isscalar(heading)
  error('Heading must be a scalar representing an angle!')
end

if nargin < 5
  debug = false;
end

switch class(vehicle)
  case 'UTMQuad4D'
    u = getToPose_qr(obj, vehicle, position, heading, debug);
    
  otherwise
    error('This function is only implemented for the Quadrotor class!')
end
end

% =======================================
% ============== QUADROTOR ==============
% =======================================
function u = getToPose_qr(obj, vehicle, position, heading, debug)
%% Compute liveness value
% Get state in the target heading frame
base_pos = rotate2D(vehicle.getPosition - position, -heading);
base_vel = rotate2D(vehicle.getVelocity, -heading);
base_x = [base_pos(1); base_vel(1); base_pos(2); base_vel(2)];

% Evaluate value function
valuex = eval_u(obj.qr_abs_target_V.g, obj.qr_abs_target_V.data, base_x);

%% Check if vehicle has arrived
if valuex <= obj.ttt
  if debug
    disp('Arrived at target')
  end
  vehicle.waypoints = [];
  u = [];
  return;
end

%% Check if vehicle is in the band
% The band is where the vehicle could get stuck due to numerical
% integration errors
x0left = -1.5*obj.qr_abs_target_V.g.dx(1);
y0left = 10;

x0right = obj.qr_abs_target_V.g.dx(1);
y0right = 10 - 1.5*obj.qr_abs_target_V.g.dx(2);

if in_band([x0left y0left], [x0right y0right], ...
    vehicle.uMax, base_x([1 2])) && base_x(2) >= -2 && ...
    abs(base_x(3)) <= 3 && abs(base_x(4)) <= 2
  if debug
    disp(['In band, value = ' num2str(valuex)])
  end
  
  % x control simply accelerates; y control follows optimal Hamiltonian
  base_p = calculateCostate( ...
    obj.qr_abs_target_V.g, obj.qr_abs_target_V.grad, base_x);
  ux = vehicle.uMax;
  uy = (base_p(4)>=0)*vehicle.uMin + (base_p(4)<0)*vehicle.uMax;
  
  % Rotate back to vehicle frame
  u = rotate2D([ux; uy], heading);
  return;
end

%% Check if vehicle is in reachable set
if valuex <= obj.rtt
  if debug
    disp(['Inside reachable set, value = ' num2str(valuex)])
  end
  
  % Follow control given by optimal Hamiltonian
  base_p = calculateCostate( ...
    obj.qr_abs_target_V.g, obj.qr_abs_target_V.grad, base_x);
  ux = (base_p(2)>=0)*vehicle.uMin + (base_p(2)<0)*vehicle.uMax;
  uy = (base_p(4)>=0)*vehicle.uMin + (base_p(4)<0)*vehicle.uMax;
  
  % Rotate back to vehicle frame
  u = rotate2D([ux; uy], heading);
  
  return;
end

%% If vehicle is in fact outside of reachable set
% Plan a straight path to target position
if debug
  disp('Outside reachable set')
end

% Amount behind the target set to aim for (based on target heading)
behind_amount = 0;
if isempty(vehicle.waypoints)
  behind_target = position - behind_amount*[cos(heading); sin(heading)];
  vehicle.waypoints = ...
    Linpath(vehicle.getPosition, behind_target, obj.hw_speed);
end
u = vehicle.followPath(vehicle.waypoints);

end

function in = in_band(left, right, u, state)
% Given y, evaluate x to see if it's between the band defined by the left
% and right optimal curves for the double integrator
%
% Inputs: left, right - the left and right points (x(0), y(0)), i.e. the
%                       final points for the left and right switching
%                       curves
%         u           - maximum acceleration
%         state       - the point (x,y) to be checked
% Output: in          - boolean indicating whether state is between left
%                       and right
%

in = false;

if state(1) > opt_curve_x(left(1), left(2), u, state(2)) && ...
    state(1) < opt_curve_x(right(1), right(2), u, state(2))
  in = true;
end

end

function x = opt_curve_x(x0, y0, u, y)
% Takes the final point (x0, y0), acceleration u, and a query value y as
% input, and outputs the corresponding value x that lies on the optimal
% curve determined by (x0, y0)

% Parametric form of the curve:
% x = 0.5 * u * t.^2 + y0*t + x0;
% y = u*t + y0;
% Take y as input and solve for x

t = (y-y0)/u;
x = 0.5 * u * t.^2 + y0*t + x0;
end

% =============================================
% ============ END OF QUADROTOR ===============
% =============================================