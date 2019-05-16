function assignment()
%% Should we compute the trajectory?
compTraj = false;

%% Grid
grid_min = [-2; -2.5; -pi]; % Lower corner of computation domain
grid_max = [5; 4; pi];    % Upper corner of computation domain
N = [45; 45; 35];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
toler = [0.2; 0.2; pi/10];
goal = [2; 2; 3*pi/8];
lower = goal - toler;
upper = goal + toler;
data0 = shapeRectangleByCorners(g, lower, upper);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 10; %10
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 0.4;
wMax = 0.5;
dmax = [0.05; 0.05; 0.005];
% do dStep1 here

% control trying to min or max value function?
uMode = 'min';
dMode = 'max';
% do dStep2 here


%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
dCar = DubinsCar([0, 0, 0], wMax, speed, dmax); %do dStep3 here

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;
schemaData.dMode = dMode;
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

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'minVWithTarget', HJIextraArgs);
save('a1q4.mat', 'tau', 'g', 'data')
%% Plotting Level sets
% To visualize time slices, pick the correct 4th index
figure(2);
load('a1q4.mat');
hold on;
ind = find(tau==0);
visSetIm(g, data(:,:,:,ind),'blue',0); %visSetIm(g, data, color, level, sliceDim, applyLight)
end
