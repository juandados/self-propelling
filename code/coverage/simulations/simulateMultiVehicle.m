function simulateMultiVehicle(save_figures, fig_formats)
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
% Setup speed limit
tfm.speed_limit = 10;
% collision radius
tfm.cr = 5;
%% Domain
vertX = [-10,-10,10,10];
vertY = [-10,10,10,-10];
dm = TargetDomain(vertX, vertY);
tfm.addDomain(dm);

% plot
f = figure;
domainPlot(dm);
hold on

f.Children.FontSize = 16;
f.Position(1:2) = [200 200];
f.Position(3:4) = [1000 750];
f.Color = 'white';

%% Quadrotors
%(1 vehicles)
n = 4;
xs = 50*rand(n,1);
ys = 50*rand(n,1);
vq = [0.1 0];

for j = 1:length(xs)
  q = UTMQuad4D([xs(j) vq(1) ys(j) vq(2)]);
  tfm.regVehicle(q);
end

colors = lines(length(tfm.aas));
for j = 1:length(tfm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.arrowLength = 5;
  tfm.aas{j}.plotPosition(extraArgs);
end

%xlim([-50 50])
%ylim([-50 50])
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

%save('bla.mat', 'tfm');
%% Integration
tMax = 25;
t = 0:tfm.dt:tMax;

u = cell(size(tfm.aas));
for i = 1:length(t)
  [safe, uSafe] = tfm.checkAASafety;
  class(uSafe)
  uCoverage = tfm.coverageCtrl;
  for j = 1:length(tfm.aas)
    if safe(j)
      u{j} = uCoverage{j};
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

%tfm.printBreadthFirst;
end
