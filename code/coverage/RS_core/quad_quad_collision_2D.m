function [data, g, tau] = quad_quad_collision_2D(d, speed, visualize)
% function [data, g2D, tau] = quad2Dcollision(d, speed, visualize)
%
% Computes collision reachable set for 4D relative quadrotor dynamics by
% first computing 2D reachable sets in each axis and the reconstructing the
% 4D reachable set
%
% The relative coordinate dynamics in each axis is
% \dot x_r = v_r (= ve - vp)
% \dot v_r = ue - up
%
% where input up trying to hit the target and
%       input ue trying to avoid the target.
%       
%
% Inputs:
%   d         - separation distance (default = 5)
%   visualize - whether to visualize the 2D reachable set (default = 1)
%
% Outputs:
%   dataC - 2D reachable sets in the x and y directions
%   g2D   - 2D grids in the x and y directions
%   tau   - time vector
%
% Mo Chen, 2015-07-14

%--------------------------------------------------------------------------
% You will see many executable lines that are commented out.
%   These are included to show some of the options available; modify
%   the commenting to modify the behavior.

if nargin < 1
  d = 5;
end

if nargin < 2
  speed = 10;
end

if nargin < 3
  visualize = 1;
end


%---------------------------------------------------------------------------
% Integration parameters.
tMax = 3;                    % End time.
plotSteps = 1;               % How many intermediate plots to produce?
t0 = 0;                      % Start time.
singleStep = 1;              % Plot at each timestep (overrides tPlot).

% Period at which intermediate plots should be produced.
tPlot = (tMax - t0) / (plotSteps - 1);

% How close (relative) do we need to get to tMax to be considered finished?
small = 100 * eps;

% What kind of dissipation?
dissType = 'global';

%---------------------------------------------------------------------------
% Problem Parameters.
upMax = 3;
upMin = -3;
ueMax = 3;
ueMin = -3;

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 81;

% Create the grid.
g.dim = 2;                           % Number of dimensions
g.min = [-80; -2.1*speed ];     % Bounds on computational domain
g.max = [ 80;  2.1*speed ];
g.bdry = @addGhostExtrapolate;
g.N = [ Nx; ceil(1.5*Nx/(g.max(1)-g.min(1))*(g.max(2)-g.min(2)))];
g = processGrid(g);

% ----------------- Target -----------------
% Below separation distance for any relative velocity
data = shapeRectangleByCorners(g, [-d; -inf], [d; inf]);
%---------------------------------------------------------------------------
% Set up spatial approximation scheme.
schemeFunc = @termLaxFriedrichs;
schemeData.hamFunc = @HamFunc;
schemeData.partialFunc = @PartialFunc;

% The Hamiltonian and partial functions need problem parameters.
schemeData.upMax = upMax;
schemeData.upMin = upMin;
schemeData.ueMax = ueMax;
schemeData.ueMin = ueMin;

%---------------------------------------------------------------------------
% Choose degree of dissipation.
switch(dissType)
  case 'global'
    schemeData.dissFunc = @artificialDissipationGLF;
  case 'local'
    schemeData.dissFunc = @artificialDissipationLLF;
  case 'locallocal'
    schemeData.dissFunc = @artificialDissipationLLLF;
  otherwise
    error('Unknown dissipation function %s', dissFunc);
end

%---------------------------------------------------------------------------
accuracy = 'veryHigh';

% Set up time approximation scheme.
integratorOptions = odeCFLset('factorCFL', 0.9, 'stats', 'off');

% Choose approximations at appropriate level of accuracy.
switch(accuracy)
  case 'low'
    schemeData.derivFunc = @upwindFirstFirst;
    integratorFunc = @odeCFL1;
  case 'medium'
    schemeData.derivFunc = @upwindFirstENO2;
    integratorFunc = @odeCFL2;
  case 'high'
    schemeData.derivFunc = @upwindFirstENO3;
    integratorFunc = @odeCFL3;
  case 'veryHigh'
    schemeData.derivFunc = @upwindFirstWENO5;
    integratorFunc = @odeCFL3;
  otherwise
    error('Unknown accuracy level %s', accuracy);
end

if(singleStep)
  integratorOptions = odeCFLset(integratorOptions, 'singleStep', 'on');
end

%---------------------------------------------------------------------------
% Initialize Display
if visualize
  f = figure;

  [~, h1] = contour(g.xs{1}, g.xs{2}, data, [0 0],'r'); hold on
  contour(g.xs{1}, g.xs{2}, data, [0 0],'r--');
  
  xlabel('x')
  ylabel('v')
  
  drawnow;
end
% return
%---------------------------------------------------------------------------
% Loop until tMax (subject to a little roundoff).
tNow = t0;
tau = tNow;
while(tMax - tNow > small * tMax)
  % How far to step?
  tSpan = [ tNow, min(tMax, tNow + tPlot) ];
  
  % Reshape data array into column vector for ode solver call.
  y0 = data(:,:,end);
  y0 = y0(:);
  schemeData.grid = g;
  [t, y] = feval(integratorFunc, schemeFunc, tSpan, y0,...
    integratorOptions, schemeData);
  data = cat(3, data, reshape(y, g.shape));
  
  tNow = t(end);
  tau = cat(1, tau, tNow);
  
  % Create new visualization.
  if visualize
    delete(h1)
      [~, h1] = contour(g.xs{1}, g.xs{2}, data(:,:,end), [0 0],'r'); hold on

    title(['t=' num2str(tNow)]);
  end
  
  drawnow;
end

end

%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function hamValue = HamFunc(t, data, deriv, schemeData)
% HamFunc: analytic Hamiltonian for collision avoidance.
%
% hamValue = HamFunc(t, data, deriv, schemeData)
%
% This function implements the hamFunc prototype for the three dimensional
%   aircraft collision avoidance example (also called the game of
%   two identical vehicles).
%
% It calculates the analytic Hamiltonian for such a flow field.
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   deriv	 Cell vector of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%
%   hamValue	 The analytic hamiltonian.
%
% schemeData is a structure containing data specific to this Hamiltonian
%   For this function it contains the field(s):


checkStructureFields(schemeData, 'upMax','upMin', 'ueMax','ueMin', 'grid');

grid = schemeData.grid;

upMax = schemeData.upMax;
upMin = schemeData.upMin;
ueMax = schemeData.ueMax;
ueMin = schemeData.ueMin;


% Dynamics:
% \dot{x}_r = vr
% \dot{v}_r = ue - up

% Hamiltonian
% H = min_up max_ue p1 * x2 + p2 * ue - p2 * up

% quadrotor 1 minimizes value, quadrotor 2 maximizes value
hamValue = deriv{1} .* grid.xs{2} + ...
  (deriv{2}>=0) .* (deriv{2}) * ueMax + ...
  (deriv{2}<0) .* (deriv{2}) * ueMin + ...
  (-deriv{2}>=0) .* (-deriv{2}) * upMin + ...
  (-deriv{2}<0) .* (-deriv{2}) * upMax;

% backwards reachable set
hamValue = -hamValue;
end

%---------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%---------------------------------------------------------------------------
function alpha = PartialFunc(t, data, derivMin, derivMax, schemeData, dim)
% PartialFunc: Hamiltonian partial fcn.
%
% It calculates the extrema of the absolute value of the partials of the
%   analytic Hamiltonian with respect to the costate (gradient).
%
% Parameters:
%   t            Time at beginning of timestep (ignored).
%   data         Data array.
%   derivMin	 Cell vector of minimum values of the costate (\grad \phi).
%   derivMax	 Cell vector of maximum values of the costate (\grad \phi).
%   schemeData	 A structure (see below).
%   dim          Dimension in which the partial derivatives is taken.
%
%   alpha	 Maximum absolute value of the partial of the Hamiltonian
%		   with respect to the costate in dimension dim for the
%                  specified range of costate values (O&F equation 5.12).
%		   Note that alpha can (and should) be evaluated separately
%		   at each node of the grid.


checkStructureFields(schemeData, 'ueMax','ueMin', 'upMax','upMin', 'grid');

grid = schemeData.grid;


ueMax = schemeData.ueMax;
ueMin = schemeData.ueMin;
upMax = schemeData.upMax;
upMin = schemeData.upMin;

switch dim
  case 1
    alpha = abs(grid.xs{2});
    
  case 2
    alpha = max(abs([ueMin ueMax])) + max(abs([upMin upMax]));
    
  otherwise
    error([ 'Partials only exist in dimensions 1-2' ]);
end
end