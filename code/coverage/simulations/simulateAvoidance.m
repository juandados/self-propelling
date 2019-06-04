function simulateAvoidance(save_figures, fig_formats)
% Simulates 2 quadrotors avoiding each other

addpath(genpath('..'))

if nargin < 1
  save_figures = false;
end

if nargin < 2
  fig_formats = {'png'};
end

%% TFM
tfm = TFM;
tfm.computeRS('qr_qr_safe_V');
% Highway speed
tfm.hw_speed = 10;
% collision radius
tfm.cr = 5;
%% Domain
%dm = Domain();
%tfm.addDomain(hw);

% plot
f = figure;
%dm.lpPlot;
hold on

f.Children.FontSize = 16;
f.Position(1:2) = [200 200];
f.Position(3:4) = [1000 750];
f.Color = 'white';

%% Quadrotors
%(1 vehicles)
n = 1;
xs = zeros(n,1);
ys = zeros(size(xs));
theta = -pi/2*rand;
vq = [10 0];
vq = rotate2D(vq, theta);

for j = 1:length(xs)
  q = UTMQuad4D([xs(j) vq(1) ys(j) vq(2)]);
  tfm.regVehicle(q);
end

% Intruder
pin = [250 50];
vin = [10 0];
vin = rotate2D(vin, 9*pi/8);
pin = rotate2D(pin, theta);
vin = rotate2D(vin, theta);
intruder = UTMQuad4D([pin(1) vin(1) pin(2) vin(2)]);
tfm.regVehicle(intruder);

colors = lines(length(tfm.aas));
for j = 1:length(tfm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.arrowLength = 5;
  tfm.aas{j}.plotPosition(extraArgs);
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
tMax = 25;
t = 0:tfm.dt:tMax;

u = cell(size(tfm.aas));
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;
  safe(length(tfm.aas)) = 1;
  for j = 1:length(tfm.aas)
    if safe(j)
      u{j} = controlLogic(tfm, tfm.aas{j});
    else
      u{j} = uSafe{j};
    end
    
    tfm.aas{j}.updateState(u{j}, tfm.dt);
    tfm.aas{j}.plotPosition;
    
  end
  
  % Plot reachable set
  tfm.aas{1}.plot_safe_V(tfm.aas{end}, tfm.qr_qr_safe_V, tfm.safetyTime)
    
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

tfm.printBreadthFirst;
end

function u = controlLogic(tfm, veh)
if strcmp(veh.q, 'Free')
  u = [0; 0];
  return
end
end
