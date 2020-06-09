function simulateSingleIntruderSigleDubinsAcc(save_figures, fig_formats)
% Simulates 2 quadrotors avoiding each other

addpath('..');

if nargin < 1
  save_figures = false;
end

if nargin < 2
  fig_formats = {'png'};
end

%% TM
% ---- Traffic Manager ----
tm = TMDubins;
% setup speed limit
tm.speedLimit = 100;
% Min Speed
tm.speedMin = 0;
% collision radius
tm.cr = 2;
% safety time (it will be safe during the next st seconds)
tm.safetyTime = 15;
% maximum force for vehicles
tm.uThetaMax = pi/10; %same as omegaMax or wMax in model
tm.uVMax = 3; %same as aMax in model

% compute reachable set
tm.computeRS('db_db_safe_V_circle');

% plot
f = figure;
%dm.lpPlot;
hold on

f.Children.FontSize = 16;
f.Position(1:2) = [200 200];
f.Position(3:4) = [1000 750];
f.Color = 'white';

%% Dubins Cars
%(1 vehicles)
n = 1;
%r = 1;
r = 0.07;
%r = 0.05;
xs = zeros(n,1);
ys = zeros(size(xs));
%theta = -pi/2*rand;
theta = -pi*rand;
%v0 = 7*rand;
v0 = 5;

% Main Vehicle
for j = 1:length(xs)
  q = UTMDubinsCarAccelerated([xs(j) ys(j) theta v0], tm.uThetaMax, ... 
      tm.uVMax, tm.speedLimit, tm.speedMin);
  tm.regVehicle(q);
end

% Intruder
pin = r*[250 50];
vin = v0;
pin = rotate2D(pin, theta);
intruder = UTMDubinsCarAccelerated([pin(1) pin(2) theta+9*pi/8 vin], tm.uThetaMax, ...
    tm.uVMax, tm.speedLimit, tm.speedMin);
tm.regVehicle(intruder);

colors = lines(length(tm.aas));
for j = 1:length(tm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.ArrowLength = 5;
  tm.aas{j}.plotPosition(extraArgs);
end

xlim([-50 200])
ylim([-200 50])
title('t=0')
axis square
drawnow

if save_figures
  fig_dir = [fileparts(mfilename('fullpath')) '\' mfilename '_figs'];
  if ~exist(fig_dir, 'dir')
    cmd = ['mkdir ' fig_dir];
    system(cmd)
  end
  
  for ii = 1:length(fig_formats)
    if strcmp(fig_formats{ii}, 'png')
      export_fig([fig_dir '/0'], '-png', '-m2', '-transparent')
    end
    
    if strcmp(fig_formats{ii}, 'pdf')
      export_fig([fig_dir '/0'], '-pdf', '-m2', '-transparent')
    end
    
    if strcmp(fig_formats{ii}, 'fig')
      savefig([fig_dir '/0.fig']);
    end
  end
end

%% Integration
tMax = 50;
t = 0:tm.dt:tMax;

u = cell(size(tm.aas));
for i = 1:length(t)
  disp(['time:', num2str(t(i))]);
  [safe, uSafeOptimal] = tm.checkAASafety;
  safe(length(tm.aas)) = 1;
  for j = 1:length(tm.aas)
    if safe(j)
      u{j} = controlLogic(tm, tm.aas{j});
    else
      u{j} = uSafeOptimal{j};
    end
    uu = u{j};
    tm.aas{j}.updateState(uu, tm.dt);
    tm.aas{j}.plotPosition;
    
  end
    
  title(['t=' num2str(t(i))])
  drawnow

  if save_figures
    for ii = 1:length(fig_formats)
      if strcmp(fig_formats{ii}, 'png')
        export_fig([fig_dir '/' num2str(i)], '-png', '-m2', '-transparent')
      end

      if strcmp(fig_formats{ii}, 'pdf')
        export_fig([fig_dir '/' num2str(i)], '-pdf', '-m2', '-transparent')
      end

      if strcmp(fig_formats{ii}, 'fig')
        savefig([fig_dir '/' num2str(i) '.fig']);
      end
    end
  end
end

%tm.printBreadthFirst;
end

function u = controlLogic(tm, veh)
if strcmp(veh.q, 'Free')
  u = [0; 0];
  return
end
end
