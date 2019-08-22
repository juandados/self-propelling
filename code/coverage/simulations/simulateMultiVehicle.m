function simulateMultiVehicle(recordVideo)
% Simulates 2 quadrotors avoiding each other
rng(2);

addpath('..');

if nargin < 1
  recordVideo = false;
end

% ---- Traffic Manager ----

tm = TM;
% compute reachable set
tm.computeRS('qr_qr_safe_V_circle');
% setup speed limit
tm.speed_limit = 10;
% collision radius
tm.cr = 1;

% domain setup
domainType = 'square';
switch domainType
    case 'square'
        vertX = [-50,-50, 50, 50];
        vertY = [-50, 50, 50, -50];
    case 'L'
        vertX = [-50, 50, 50, 0, 0, -50];
        vertY = [-50, -50, 0, 0, 50, 50];
    case 'polygon'
        vertX = [-50, 50, 0, 0];
        vertY = [-50, 0, 0, 50];
    case 'triangle'
        n = 3;
        t = linspace(0, 2*pi*(1-1/n), n);
        vertX = 50 * sin(t);
        vertY = 50 * cos(t);
    case 'pentagon'
        n = 5;
        t = linspace(0, 2*pi*(1-1/n), n);
        vertX = 50 * sin(t);
        vertY = 50 * cos(t);
end
domain = TargetDomain(vertX, vertY);
tm.addDomain(domain);

% figure setup
f = figure;
domain.domainPlot('blue', 'red');
hold on;
f.Children.FontSize = 9;
f.Position(1:2) = [200 200];
f.Position(3:4) = [1000 750];
f.Color = 'white';
xlim([-250 250]);
ylim([-250 250]);
title('t=0');
axis square;

% ---- Quadrotors ----

% random initial states
n = 9;
px = 50 * rand(n,1);
py = 50 * rand(n,1);
vx = 0.1 * rand(n,1);
vy = 0.1 * rand(n,1);
maxForce = 3;
minForce = -3;

% registering vehicles in traffic manager
for j = 1:length(px)
  q = UTMQuad4D([px(j) vx(1) py(j) vy(2)], minForce, maxForce);
  tm.regVehicle(q);
end

% plotting initial conditions
colors = lines(length(tm.aas));
for j = 1:length(tm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.ArrowLength = 0; %j prev 1
  extraArgs.LineWidth = 0.01; %j
  tm.aas{j}.plotPosition(extraArgs);
end

drawnow

if recordVideo
    nFrames = 20;
    vidObj = VideoWriter([domainType,'.avi']);
    vidObj.Quality = 100;
    vidObj.FrameRate = 10;
    open(vidObj);
end

% Time integration
tMax = 60;
t = 0:tm.dt:tMax;

u = cell(size(tm.aas));
for i = 1:length(t)
  [safe, uSafeOptimal, uSafeRight, uSafeLeft] = tm.checkAASafety;
  uCoverage = tm.coverageCtrl;
  for j = 1:length(tm.aas)
    if safe(j)
      u{j} = uCoverage{j};
    else
      %projection aug 22 2019
      uC = uCoverage{j}';
      uO = uSafeOptimal{j};
      uR = uSafeRight{j};
      uL = uSafeLeft{j};
      s = uL - uR;
      d = uR - uC;
      zref = s(2) * (uO(1) - uR(1)) - s(1) * (uO(2) - uR(2));
      zcomp = s(2) * (uC(1) - uR(1)) - s(1) * (uC(2) - uR(2));
      if zref * zcomp < 0
          u{j} = -(d' * s / norm(s)^2)*s + uR;
          if norm(u{j}) > umax
              ctrls = [uR, uL];
              dists = [norm(uC-uR), norm(uC-uL)];
              [minDist, ind] = min(dists);
              u{j} = ctrls(:,ind);
          end
      else
          u{j} = uC;
      end
      % using only optimal
      %u{j} = uO;
    end  
    tm.aas{j}.updateState(u{j}, tm.dt);
    tm.aas{j}.plotPosition;
    % Plot reachable set
    %tm.aas{j}.plot_safe_V(tm.aas{end}, tm.qr_qr_safe_V, tm.safetyTime)
  end
  title(['t=' num2str(t(i))])
  drawnow
  if recordVideo
      writeVideo(vidObj, getframe(gca));
  end
end
end
