function [data, g, tau, grad] = quad_quad_collision_4D(d, speed, visualize)
%
% Computes collision reachable set for 4D relative quadrotor dynamics for a 
% circular collision set by
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
%   data - 4D reachable sets in the x and y directions
%   g   - 4D grids in the x and y directions
%   tau   - time vector
%
% Juan Chacon, 2019-aug-05

if nargin < 1
  d = 5;
end

if nargin < 2
  speed = 10;
end

if nargin < 3
  visualize = 1;
end

%--AS IN TUTORIAL----------------------------------------------------------
% Problem Parameters.
aMax = 3; % aMax: x- and y-acceleration bound for vehicle A
bMax = 3; % bMax: x- and y-acceleration bounds for vehicle B

%---------------------------------------------------------------------------
% Approximately how many grid cells?
%   (Slightly different grid cell counts will be chosen for each dimension.)
Nx = 51;

% Create the grid.
g.dim = 4; % Number of dimensions
% ???? The limits could be chosen diferent?
g.min = [-20; -15; -20; -15];     % Bounds on computational domain
g.max = [20; 15; 20; 15];
g.bdry = @addGhostExtrapolate;
g.N = [Nx; Nx; Nx; Nx];
% ???? quad_quad_collision_2D uses g = processGrid(g) instead
%g = createGrid(grid_min, grid_max, N);
g = createGrid(g.min, g.max, g.N); % there are not periodic state space dim

% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data = shapeCylinder(g, [2, 4], [0; 0; 0; 0], d);

% time vector
t0 = 0;
dt = 0.05;
tMax = 2*dt;
tau = t0:dt:tMax;

% control trying to min or max value function?
uMode = 'max'; % for collision avoidance
dMode = 'min'; % opposite to uMode
minWith = 'minVWithTarget'; % for BRT no BRS - Tube (not set)

% Define dynamic system
%obj = Quad4DCAvoid(x, aMax, bMax, dxMax, dyMax, dims)

% ???? Does the state x affect the complutation of data?
dQuad = Quad4DCAvoid([0, 0, 0, 0], aMax*[1, 1], bMax*[1, 1]);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dQuad;
schemeData.accuracy = 'veryHigh'; % set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode; % Disturbance is used or not?

% Compute value function
HJIextraArgs.keepLast = false;
HJIextraArgs.visualize = visualize; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
% ???? the output should be data(:,:,:,:,end) ?
[data, tau, ~] = ...
  HJIPDE_solve(data, tau, schemeData, minWith, HJIextraArgs);

grad = computeGradients(g, data);
           
end
