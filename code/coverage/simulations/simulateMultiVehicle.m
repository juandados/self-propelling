function simulateMultiVehicle(save_figures, fig_formats)
% Simulates 2 quadrotors avoiding each other
rng(2);

addpath(genpath('..'))

if nargin < 1
  save_figures = false;
end

if nargin < 2
  fig_formats = {'png'};
end

%% TM
tm = TM;
tm.computeRS('qr_qr_safe_V');
% Setup speed limit
tm.speed_limit = 10;
% collision radius
tm.cr = 1;
%% Domain
vertX = [-50,-50,50,50];
vertY = [-50,50,50,-50];
domain = TargetDomain(vertX, vertY);
tm.addDomain(domain);

% plot
f = figure;
domainPlot(domain);
hold on

f.Children.FontSize = 16;
f.Position(1:2) = [200 200];
f.Position(3:4) = [1000 750];
f.Color = 'white';

%% Quadrotors
%(1 vehicles)
n = 16;
xs = 50*rand(n,1);
ys = 50*rand(n,1);
vq = [0.1 0];

for j = 1:length(xs)
  q = UTMQuad4D([xs(j) vq(1) ys(j) vq(2)], -0.75*9.8, 0.75*9.8);
  tm.regVehicle(q);
end

colors = lines(length(tm.aas));
for j = 1:length(tm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.ArrowLength = 0; %j prev 1
  extraArgs.LineWidth = 0.01; %j
  tm.aas{j}.plotPosition(extraArgs);
end

xlim([-250 250]);
ylim([-250 250]);
title('t=0');
axis square;
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
tMax = 150;
t = 0:tm.dt:tMax;

u = cell(size(tm.aas));
for i = 1:length(t)
  [safe, uSafe] = tm.checkAASafety;
  uCoverage = tm.coverageCtrl;
  for j = 1:length(tm.aas)
    if safe(j) || false
      u{j} = uCoverage{j};
    else
      %disp("using safety ctrl")
      u{j} = uSafe{j};
    end
    
    tm.aas{j}.updateState(u{j}, tm.dt);
    tm.aas{j}.plotPosition;
    % Plot reachable set
    %tm.aas{j}.plot_safe_V(tm.aas{end}, tm.qr_qr_safe_V, tm.safetyTime)
  
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
