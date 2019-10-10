function tm = simulateMultiVehicle(recordVideo, saveFigures)
% Simulates 2 quadrotors avoiding each other
rng(2);

addpath('..');

if nargin < 1
  recordVideo = false;
end

if nargin < 2
  saveFigures = false;
end

screenShotsCount = 10;
% ---- Traffic Manager ----
tm = TM;
% setup speed limit
tm.speedLimit = 10;
% collision radius
tm.cr = 2;
% safety time (it will be safe during the next st seconds)
tm.safetyTime = 5;
% maximum force for vehicles
tm.uMax = 3;

% compute reachable set
%tm.computeRS('qr_qr_safe_V_circle');
movingBoundary = false;
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
        vertX = -[-50, 50, 0, 0];
        vertY = -[-50, 0, 0, 50];
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
    case 'trianglePaper'
        r = 0.25;
        ns = 3;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t)*r;
        vertY = 50 * cos(t)*r;
        vertY = vertY - (max(vertY)+min(vertY))/2;
%         r = 0.20;
%         vertX = [-50,-50, 50, 50]*r;
%         vertY = [-50, 50, 50, -50]*r;
%         tMax = 40;
%         safetyTime = 5;
%         tm.speedLimit = 10;
%         n=16
%         initialConfig = 'line';
    case 'squarePaper'
        r = 0.20;
        vertX = [-50,-50, 50, 50]*r;
        vertY = [-50, 50, 50, -50]*r;
%         r = 0.20;
%         vertX = [-50,-50, 50, 50]*r;
%         vertY = [-50, 50, 50, -50]*r;
%         tMax = 60;
%         safetyTime = 5;
%         tm.speedLimit = 10;
%         n=16
%         initialConfig = 'line';
    case 'arrowPaper'
        r = 0.3;
        vertX = r*[50, -50, 0, 0];
        vertY = r*[50, 0, 0, -50];
        movingBoundary = true;
        vDomain = 0.3;
%         r = 0.3;
%         vertX = r*[50, -50, 0, 0];
%         vertY = r*[50, 0, 0, -50];
%         movingBoundary = true;
%         vDomain = 0.3;
%         tMax = 100;
%         safetyTime = 5;
%         tm.speedLimit = 10;
%         n=9
%         initialConfig = 'arrowPaper';
%         extraArgs.tailSize = -1
%         avoidance = true;
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
scale = 3;
xlim([min(vertX) max(vertX)]*scale);
ylim([min(vertY) max(vertY)]*scale);

title('t=0');
axis square;
% ---- Quadrotors ----

n = 9;
initialConfig = 'line';
if strcmp(initialConfig,'line')
    px = linspace(-30,30,n)';
    py = -15*ones(n,1);
elseif strcmp(initialConfig, 'arrowPaper')
    xx = linspace(-10,10,n);
    px = xx-10;
    py = -1*xx-10;
    xlim([-20, 60]);
    ylim([-20, 60]);
elseif strcmp(initialConfig, 'random')
    px = -10 * rand(n,1);
    py = -10 * rand(n,1);
end
vx = 0.1 * rand(n,1);
vy = 0.1 * rand(n,1);

% registering vehicles in traffic manager
uMin = -tm.uMax;
uMax = tm.uMax;
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
  extraArgs.tailSize = 30; % -1 for showing the whole tail;
  tm.aas{j}.plotPosition(extraArgs);
end

drawnow

% Time integration
tm.dt = 0.1;
tMax = 100;
t = 0:tm.dt:tMax;

avoidance = true;

% setting up figure saving
if saveFigures
    dir = ['figures/', domainType,' n',num2str(n),' ',datestr(datetime('now'))];
    mkdir(dir);
    %saving meta data
    fileID = fopen([dir,'/metadata.txt'],'w');
    fprintf(fileID, 'domain type: %s\n', domainType);
    fprintf(fileID, 'initial config: %s\n', initialConfig);
    fprintf(fileID, 'number of vehicles: %d\n', n);
    fprintf(fileID, 'avoidance: %f \n', avoidance);
    fprintf(fileID, 'speedLimit: %f\n', tm.speedLimit);
    fprintf(fileID, 'collision radius: %f\n', tm.cr);
    fprintf(fileID, 'safety time: %f\n', tm.safetyTime);
    fprintf(fileID, 'u max: %f\n', tm.uMax);
    fprintf(fileID, 'tmax: %f \n', tMax);
end

% setting up video recorder
if recordVideo
    vidObj = VideoWriter([dir, '/movie.avi']);
    vidObj.Quality = 100;
    vidObj.FrameRate = 10;
    open(vidObj);
end

u = cell(size(tm.aas));
for i = 1:length(t)
  %update domain if moving domaing
  if movingBoundary
    domain = TargetDomain(vertX+t(i)*vDomain, vertY+t(i)*vDomain);
    hold off;
    domain.domainPlot('blue', 'red');
    hold on;
    f.Color = 'white';
    xlim([-20, 60]);
    ylim([-20, 60]);
    axis square;
    tm.addDomain(domain);
  end
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
    extraArgs.Color = colors(j,:);
    tm.aas{j}.plotPosition(extraArgs);
    %tm.aas{j}.plotPosition;
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
  if saveFigures && mod(i-1,floor(length(t)/screenShotsCount))==0
      savefig([dir,'/',num2str(t(i)),'.fig'])
  end
end
  %collision count
  collisionCount = 0;
  for i=1:n
    for j=i+1:n
        collisionCount = collisionCount + sum(abs(diff(tm.collisions{i,j})))/2;
    end
  end
% disp(['max vx: ', num2str(max(velx(:))), ' min vx: ', num2str(min(velx(:)))]);
% disp(['max vy: ', num2str(max(vely(:))), ' min vy: ', num2str(min(vely(:)))]);
disp(['collision count: ', num2str(collisionCount)]);
if saveFigures
    fprintf(fileID, 'collision count: %d\n', collisionCount);
    fclose(fileID);
end
%disp(['unsafe count: ', num2str(tm.unsafeCount)]);
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