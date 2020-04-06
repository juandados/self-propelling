%% Grid
grid_min = [-10, -5, -10, -5]; % Lower corner of computation domain
grid_max = [10, 5, 10, 5];
% Upper corner of computation domain
N = [50; 50; 50; 50];
% 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N);
%% target set
d = 2.5;
data0 = shapeCylinder(g, [2, 4], [0; 0; 0; 0], d);
visSetIm(g,data0);
%% time vector
t0 = 0;
tMax = 0.5;
dt = 0.05;
tau = t0:dt:tMax;
%% problem parameters
uMode = 'min';
dMode = 'max';
minWith = 'minVWithTarget'; % for BRT no BRS - Tube (not set)
%% Pack problem parameters
% Define dynamic system
dQuad = Quad4DCAvoid([0, 0, 0, 0], 3*[1, 1], 3*[1, 1]);
% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dQuad;
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
%% Compute value function
HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
data = HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);
save('a1q4.mat', 'tau', 'g', 'data')
%% Computing TTR
load('a1q4.mat');
ttr = TD2TTR(g, data, tau);
%% Visualize 3D
figure;
visSetIm(g,ttr,'red',0.2);
%% Visualize 2D
figure;hold on;
[gxy, dataxy] = proj(g, ttr, [0 1 0 1], [1 1]);
visSetIm(gxy, dataxy, 'red', [0:0.1:2]);
%% USING ONESHOT
%% computing TTR
phi = mainLF();
%% Visualize 3D
figure;
visSetIm({},phi,'red',0.2);
%% Visualize 2D
% preparing Grid
r = 1;
g.min = r*[-10, -5, -10, -5];
g.max = r*[10, 5, 10, 5];
g.dx = 0.8*r*[20/49, 10/49, 20/49, 10/49];
for i = 1:4
    g.vs{i}=[g.min(i):g.dx:g.max(i)];
end

vs = [1,1.5];
ind = zeros(1,2);
val = zeros(1,2);
for i = 1:2
    [val(i),ind(i)] = min(abs(g.vs{2*i-1}-vs(i)));
end
% projecting
phi_pr = phi(:,ind(1),:,ind(2));
phi_pr = reshape(phi_pr,[size(phi_pr,1),size(phi_pr,3)]);
%figure()
%contourf(phi_pr)
ind = phi_pr >= 0.8;
phi_pr(ind) = inf;
contourf(g.vs{1},g.vs{3},phi_pr);
colorbar;
%% Improving computation
load('a1q4.mat');
g.min = [-10, -2, -10, -2];
g.max = [10, 2, 10, 2];
g.N = floor(1.5*[50, 50, 50, 50]);
g.dx = (g.max-g.min)./(g.N-1);
phi = mainLF(g.min,g.max,g.dx);
g = createGrid(g.min,g.max, g.N);
%% visualizing
figure();
visSetIm(g,phi,'red',0.2);
%% 
[gxy, dataxy] = proj(g, phi, [0 1 0 1], [1 1]);
visSetIm(gxy, dataxy, 'red', 0:0.1:2);
%visSetIm(gxy, dataxy, 'blue', 1e-3);