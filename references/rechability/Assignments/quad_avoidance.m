function quad_avoidance()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

%% Should we compute the trajectory?
compTraj = false;

%% Grid
grid_min = [-10; -5; -10; -5]; % Lower corner of computation domain
grid_max = [10; 5; 10; 5];    % Upper corner of computation domain
N = floor(0.8*[30; 20; 30; 20]);         % Number of grid points per dimension
g = createGrid(grid_min, grid_max, N); % There are no periodic
% state space dimensions
%% target set
R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeCylinder(g, [2, 4], [0; 0; 0; 0], R);
% plotting Cylinder
%visSetIm(g, data0);
%% time vector
t0 = 0;
tMax = 30;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters
% do dStep1 here
gravity = 9.8;
dMax = [0, 0];

% control trying to min or max value function?
uMode = 'max'; %avoid
% do dStep2 here
dMode = 'min'; %avoid


%% Pack problem parameters

% Define dynamic system
% obj = Quad4DCAvoid(x, aMax, bMax, dxMax, dyMax, dims)
% aMax: x- and y-acceleration bound for vehicle A
% bMax: x- and y-acceleration bounds for vehicle B
dQuad = Quad4DCAvoid([2, 0, 2, 0], 0.75*gravity*[1,1], 0.75*gravity*[1,1], 0, 0, [1 2 3 4]);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dQuad;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
%do dStep4 here

%% additive random noise
%do Step8 here
%HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
% Try other noise coefficients, like:
%    [0.2; 0; 0]; % Noise on X state
%    [0.2,0,0;0,0.2,0;0,0,0.5]; % Independent noise on all states
%    [0.2;0.2;0.5]; % Coupled noise on all states
%    {zeros(size(g.xs{1})); zeros(size(g.xs{1})); (g.xs{1}+g.xs{2})/20}; % State-dependent noise

%% If you have obstacles, compute them here

%% Compute value function

HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.keepLast = true;
HJIextraArgs.stopConverge = true;

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'minVWithTarget', HJIextraArgs);
save('quad_avoidance.mat', 'tau', 'tau', 'g', 'data')

%% Visualize Slices
load('quad_avoidance.mat')
kr = 5;
vi = kr*[-1,-1];
xi = [30, -20];
vj = kr*[1,-1];
xj = [0, 0];
vr = vi - vj;
[m, ind_vxr] =min(abs(g.vs{2,1}-vr(1)));
[m, ind_vyr] =min(abs(g.vs{4,1}-vr(2)));
% 2D Grid
grid_min = [-10; -10;]; % Lower corner of computation domain
grid_max = [10; 10;];    % Upper corner of computation domain
N = [30; 30;];         % Number of grid points per dimension
g2 = createGrid(grid_min, grid_max, N); % There are no periodic
figure(3); 
plot(xi(1), xi(2),'b.', 'markerSize', 20); hold on;
quiver(xi(1), xi(2), vi(1), vi(2), 'LineWidth', 2, 'AutoScale', 'off', 'MaxHeadSize', 2);  
plot(xj(1), xj(2),'r.', 'markerSize', 20); hold on;
quiver(xj(1), xj(2), vj(1), vj(2), 'LineWidth', 2, 'AutoScale', 'off', 'MaxHeadSize', 2);
visSetIm(g2, squeeze(data(:,ind_vxr,:,ind_vyr))); hold off;
axis([-10, 40, -30, 10]);
end
