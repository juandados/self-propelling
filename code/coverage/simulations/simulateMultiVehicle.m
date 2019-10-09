function simulateMultiVehicle(recordVideo)
% Simulates 2 quadrotors avoiding each other
rng(2);

addpath('..');

if nargin < 1
  recordVideo = false;
end

% ---- Traffic Manager ----
tm = TM;
% setup speed limit
tm.speedLimit = 10;
% collision radius
tm.cr = 2;
% safety time (it will be safe during the next st seconds)
tm.safetyTime = 4;
% maximum force for vehicles
tm.uMax = 3;

% compute reachable set
%tm.computeRS('qr_qr_safe_V_circle');

% domain setup
domainType = 'squarePaper';
switch domainType
    case 'square'
        r = 0.5;
        vertX = [-50,-50, 50, 50]*r;
        vertY = [-50, 50, 50, -50]*r;
    case 'L'
        vertX = [-50, 50, 50, 0, 0, -50];
        vertY = [-50, -50, 0, 0, 50, 50];
    case 'polygon'
        vertX = [-50, 50, 0, 0];
        vertY = [-50, 0, 0, 50];
    case 'triangle'
        ns = 3;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t);
        vertY = 50 * cos(t);
    case 'pentagon'
        ns = 5;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t);
        vertY = 50 * cos(t);
    case 'squarePaper'
        r = 0.12;
        vertX = [-50,-50, 50, 50]*r;
        vertY = [-50, 50, 50, -50]*r;   
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
scale = 8;
xlim([min(vertX) max(vertX)]*scale);
ylim([min(vertY) max(vertY)]*scale);
title('t=0');
axis square;

% ---- Quadrotors ----

% random initial states
n = 16;
initial = 'line';
if strcmp(initial,'line')
    px = linspace(-50,50,n)';
    py = -15*ones(n,1);
elseif strcmp(initial, 'random')
    px = 50 * rand(n,1);
    py = 50 * rand(n,1);
end
vx = 0.1 * rand(n,1);
vy = 0.1 * rand(n,1);

% max and min forces
uMin = -tm.uMax;
uMax = tm.uMax;

% registering vehicles in traffic manager
for j = 1:length(px)
  q = UTMQuad4D([px(j) vx(j) py(j) vy(j)], uMin, uMax);
  tm.regVehicle(q);
end

% plotting initial conditions
colors = lines(length(tm.aas));
for j = 1:length(tm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.ArrowLength = 1; %j prev 1
  extraArgs.LineWidth = 0.01; %j
  tm.aas{j}.plotPosition(extraArgs);
end

drawnow

% setting up video recorder
if recordVideo
    nFrames = 20;
    vidObj = VideoWriter([domainType,'.avi']);
    vidObj.Quality = 100;
    vidObj.FrameRate = 10;
    open(vidObj);
end

% Time integration
tm.dt = 0.1;
tMax = 150;
t = 0:tm.dt:tMax;

avoidance = false;

u = cell(size(tm.aas));
for i = 1:length(t)
  disp(['time: ', num2str(t(i))])
  [safe, uSafeOptimal, uSafeRight, uSafeLeft] = tm.checkAASafety;
  uCoverage = tm.coverageCtrl;
  for j = 1:length(tm.aas)
    if safe(j) || ~avoidance
      u{j} = uCoverage{j};
    else
      u{j} = uSafeOptimal{j};
      %u{j} = projectControl(uCoverage{j}',uSafeOptimal{j},uSafeRight{j},uSafeLeft{j});
      %disp(['Projected Controller: ', num2str(u{j}')]);
    end
    %u{j} = u{j}*((vx^2+vy^2)<100);
    tm.aas{j}.updateState(u{j}, tm.dt);
    % Adding velocity contstrain
    speed=norm([tm.aas{j}.x(2),tm.aas{j}.x(4)]);
    if speed >= tm.speedLimit
      tm.aas{j}.x(2)=tm.speedLimit*tm.aas{j}.x(2)/speed;
      tm.aas{j}.x(4)=tm.speedLimit*tm.aas{j}.x(4)/speed;
    end
    tm.aas{j}.plotPosition;
    velx(i,j) = tm.aas{j}.x(2);
    vely(i,j) = tm.aas{j}.x(4);
    % Plot reachable set
    % tm.aas{j}.plot_safe_V(tm.aas{end}, tm.qr_qr_safe_V, tm.safetyTime)
  end
  title(['t=' num2str(t(i))])
  drawnow
  if recordVideo
      writeVideo(vidObj, getframe(gca));
  end
end
    disp(['max vx: ', num2str(max(velx(:))), ' min vx: ', num2str(min(velx(:)))]);
    disp(['max vy: ', num2str(max(vely(:))), ' min vy: ', num2str(min(vely(:)))]);
    disp(['collision count: ', num2str(sum(tm.collisionCount))]);
    disp(['unsafe count: ', num2str(tm.unsafeCount)]);
end


function u = projectControl(uC,uO,uR,uL)
    s = uL - uR;
    % z values associated to the plane which is parallel to vector s and 
    % (1,1,1), it is only one of the infiniteS possible planes parallel
    % to s
    zOptimal = s(2) * (uO(1) - uR(1)) - s(1) * (uO(2) - uR(2));
    zCoverage = s(2) * (uC(1) - uR(1)) - s(1) * (uC(2) - uR(2));
    if zOptimal * zCoverage < 0
      d = uR - uC;
      % projected controller over the segment uR_uL
      u = -(d' * s / norm(s)^2)*s + uR;
      if norm(u) > 3
          ctrls = [uR, uL];
          dists = [norm(uC-uR), norm(uC-uL)];
          [minDist, ind] = min(dists);
          u = ctrls(:,ind);
      end
    else
      u = uC;
    end
end