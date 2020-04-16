function tm = simulateMultiDubins(recordVideo, saveFigures)
% Simulates 2 dubins car 
rng(30);

addpath('..');

if nargin < 1
  recordVideo = false;
end

if nargin < 2
  saveFigures = false;
end

screenShotsCount = 10;
% ---- Traffic Manager ----
tm = TMDubins;
% setup speed limit
tm.speedLimit = 10;
% Min Speed
tm.speedMin = 0.3;
% collision radius
tm.cr = 2;
% safety time (it will be safe during the next st seconds)
tm.safetyTime = 5;
% maximum force for vehicles
tm.uThetaMax = pi/10; %same as omegaMax or wMax in model
tm.uVMax = 3; %same as aMax in model


% compute reachable set
%tm.computeRS('qr_qr_safe_V_circle');

% domain setup
domainType = 'triangleInTwoLines';
switch domainType
    case 'circle'
        r = 1/100;
        ns = 100;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = r * 50 * sin(t);
        vertY = r * 50 * cos(t);
        movingBoundary = false;
    case 'diamond'
        r = 0.1;
        vertX = [-50, 0, 50, 0];
        vertY = [0, 50, 0, -50]*r;
        movingBoundary = false;
    case 'square'
        r = 0.5;
        vertX = [-50,-50, 50, 50]*r;
        vertY = [-50, 50, 50, -50]*r;
        movingBoundary = false;
    case 'L'
        vertX = [-50, 50, 50, 0, 0, -50];
        vertY = [-50, -50, 0, 0, 50, 50];
        movingBoundary = false;
    case 'polygon'
        vertX = -[-50, 50, 0, 0];
        vertY = -[-50, 0, 0, 50];
        movingBoundary = false;
    case 'triangle'
        ns = 3;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t);
        vertY = 50 * cos(t);
        movingBoundary = false;
    case 'pentagon'
        ns = 5;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t);
        vertY = 50 * cos(t);
        movingBoundary = false;
    case 'trianglePaper'
        r = 0.25;
        ns = 3;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t)*r;
        vertY = 50 * cos(t)*r;
        vertY = vertY - (max(vertY)+min(vertY))/2;
        movingBoundary = false;
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
        movingBoundary = false;
%         r = 0.20;
%         vertX = [-50,-50, 50, 50]*r;
%         vertY = [-50, 50, 50, -50]*r;
%         tMax = 60;
%         safetyTime = 5;
%         tm.speedLimit = 10;
%         n=16
%         initialConfig = 'line';
    case 'arrowPaper'
%         r = 0.3;
%         vertX = r*[50, -50, 0, 0];
%         vertY = r*[50, 0, 0, -50];
%         movingBoundary = true;
        vDomain = 0.3;
        r = 0.3;
        vertX = r*[50, -50, 0, 0];
        vertY = r*[50, 0, 0, -50];
        movingBoundary = true;
        vDomain = 0.3;
        safetyTime = 5;
        tm.speedLimit = 10;
        initialConfig = 'arrowPaper';
        extraArgs.tailSize = -1
        avoidance = true;
    case 'triangleInLine'
        vDomain = 0.3;
        r = 0.3;
        vertX = r*[50, -50, 0];
        vertY = r*[50, 0, -50];
        movingBoundary = true;
        vDomain = 0.3;
        safetyTime = 5;
        tm.speedLimit = 10;
        initialConfig = 'arrowPaper';
        extraArgs.tailSize = -1
        avoidance = true;
    case 'triangleInCircles'
%         r = 0.3;
%         vertX = r*[50, -50, 0, 0];
%         vertY = r*[50, 0, 0, -50];
%         movingBoundary = true;
        vDomain = 0.3;
        vertX = 3*[2.0, 3, 2.5];
        vertY = 3*[0, 0, sqrt(2)];
        movingBoundary = true;
        vDomain = 0.3;
        safetyTime = 5;
        tm.speedLimit = 10;
        initialConfig = 'triangleInCircles';
        extraArgs.tailSize = -1
        avoidance = true;
    case 'triangleInParabola'
        vDomain = 0.05;
        k = 5;
        vertX = k*[0, 2*sin(pi/3),0];
        vertY = k*[1, 0, -1];
        movingBoundary = true;
        safetyTime = 5;
        tm.speedLimit = 10;
        initialConfig = 'arrowPaper';
        extraArgs.tailSize = -1;
        avoidance = true;
    case 'triangleInTwoLines'
        vDomain = 0.05;
        k = 5;
        vertX = k*[0, 2*sin(pi/3),0];
        vertY = k*[1, 0, -1];
        movingBoundary = true;
        safetyTime = 5;
        tm.speedLimit = 10;
        initialConfig = 'arrowPaper';
        extraArgs.tailSize = -1;
        avoidance = true;
end
domain = TargetDomain(vertX, vertY);
tm.addDomain(domain);
% figure setup
f = figure;
domain.domainPlot('blue', 'red');
hold on;
f.Children.FontSize = 9;
f.Position(1:2) = [200 200];
%f.Position(3:4) = [350 350];
f.Position(3:4) = 3*[350 350];
f.Color = 'white';
scale = 3;
%xlim([min(vertX) max(vertX)]*scale);
%ylim([min(vertY) max(vertY)]*scale);

title('t=0');
axis square;

% ---- Adding Dubins Cars ----

% Initial formation
n = 6;
% initialConfig = 'line';
if strcmp(initialConfig,'line')
    px = linspace(-30,30,n)';
    py = -15*ones(n,1);
elseif strcmp(initialConfig, 'arrowPaper')
    xx = linspace(-10,10,n);
    px = xx-10;
    py = -1*xx-10;
elseif strcmp(initialConfig, 'triangleInCircles')
    xx = linspace(-3,3,n);
    px = xx;
    py = -0*xx-3;
elseif strcmp(initialConfig, 'random')
    px = -10 * rand(n,1);
    py = -10 * rand(n,1);
elseif strcmp(initialConfig, 'square')
    s = 1;
    px = 15*(mod(0:15,4)/3-0.5)+s*rand(1,16);
    py = 15*(floor([0:15]/4)/3-0.5)+s*rand(1,16);
end
theta = 2 * pi * rand(n,1);
%theta = 0 * pi * rand(n,1) ;
v = 0.0 * rand(n,1) ;
v = [ones(n,1), zeros(n,1)];
%v = 0 * rand(n,1) + [1; -1];

% registering vehicles in traffic manager
wMax = tm.uThetaMax;
aMax = tm.uVMax;
for j = 1:length(px)
  q = UTMDubinsCarAccelerated([px(j) py(j) theta(j) v(j)], wMax, aMax, tm.speedLimit, tm.speedMin);
  tm.regVehicle(q);
end

% plotting initial conditions
colors = lines(length(tm.aas));
for j = 1:length(tm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.ArrowLength = 3; %j prev 1
  extraArgs.LineStyle = 'none';
  extraArgs.LineWidth = 0.1; %j
  extraArgs.tailSize = 30; % -1 for showing the whole tail;
  tm.aas{j}.plotPosition(extraArgs);
end

drawnow

% Time integration
tm.dt = 0.1;
tMax = 100;
t = 0:tm.dt:tMax;

avoidance = false;

% setting up figure saving
if saveFigures
    dir = ['figuresDubins/', domainType,' n',num2str(n),' ',datestr(datetime('now'))];
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
    fprintf(fileID, 'u Theta max: %f\n', tm.uThetaMax);
    fprintf(fileID, 'u V max: %f\n', tm.uVMax);
    fprintf(fileID, 'tmax: %f \n', tMax);
end

% setting up video recorder
if recordVideo
    vidObj = VideoWriter([dir, '/movie.avi']);
    vidObj.Quality = 100;
    vidObj.FrameRate = 10;
    open(vidObj);
    ax = gca();
end


u = cell(size(tm.aas));
for i = 1:length(t)
      % juan Break point

  %update domain if  domaing
  if movingBoundary
    tl = 0;
    if t(i)>tl
        %domain = linearMotion(vertX, vertY, t(i), tl, vDomain);
        %domain = circleMotion(vertX, vertY, t(i), vDomain/(3*2.5));
        %domain = parabolicMotion(vertX,vertY,t(i),tl,vDomain);
        domain = twoLinesMotion(vertX,vertY,t(i),tl,vDomain);
        cla;
        domain.domainPlot('blue', 'red');
        f.Color = 'white';
        xlim([-40, 80]); ylim([-40, 80]); % arrow moving
        %xlim(3*[-5, 5]); ylim(3*[-5, 5]); %triangle in circle
        %axis square;
        axis square
        tm.addDomain(domain);
    end
  end
  disp(['time: ', num2str(t(i))])
  [safe, uSafeOptimal, uSafeRight, uSafeLeft] = tm.checkAASafety;
  uCoverage = tm.coverageCtrl;
  for j = 1:length(tm.aas)
    if safe(j) || ~avoidance
      u{j} = uCoverage{j};
    else
      u{j} = uSafeOptimal{j};
    end
    %u{j} = u{j}*((vx^2+vy^2)<100);
    tm.aas{j}.updateState(u{j}, tm.dt);
    % Juan: check this Adding velocity contstrain
%     speed = tm.aas{j}.x(4);
%     if speed > tm.speedLimit
%       tm.aas{j}.x(4) = tm.speedLimit;
%     end
%     if speed < tm.speedMin
%       tm.aas{j}.x(4) = tm.speedMin;
%     end
    extraArgs.Color = colors(j,:);
    tm.aas{j}.plotPosition(extraArgs);
    %tm.aas{j}.plotPosition;
    % Plot reachable set
    % tm.aas{j}.plot_safe_V(tm.aas{end}, tm.qr_qr_safe_V, tm.safetyTime)
  end
  title(['t=' num2str(t(i))])
  drawnow();
  if recordVideo
      writeVideo(vidObj, getframe(ax));
  end
  if saveFigures && (mod(i-1,floor(length(t)/screenShotsCount))==0 || ismember(t(i),[9,39,70]))
      savefig([dir,'/',num2str(t(i)),'.fig'])
  end
end
  %collision count
  collisionCount = 0;
  for i=1:n
    for j=i+1:n
%         collisionCount = collisionCount + sum(abs(diff(tm.collisions{i,j})))/2;
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

% Defining Domain Paths
function domain = parabolicMotion(vertX,vertY,s,tl,vDomain)
    heading = [0.8; 3*2*vDomain^2*s];
    theta = atan(heading(2,1)/heading(1,1));
    vert = [cos(theta),-sin(theta);sin(theta),cos(theta)]*[vertX; vertY];
    vertX = vert(1,:);
    vertY = vert(2,:);
    vertX = vertX + 0.8*s;
    vertY = vertY + 3*(vDomain*s).^2;
    domain = TargetDomain(vertX, vertY);
end

function domain = twoLinesMotion(vertX,vertY,s,tl,vDomain)
    theta_base = pi/6;
    theta = theta_base*(s<50);
    vert = [cos(theta),-sin(theta);sin(theta),cos(theta)]*[vertX; vertY];
    vertX = vert(1,:);
    vertY = vert(2,:);
    vertX = vertX + (cos(theta)*s*(s<50))+(cos(theta_base)*50+cos(theta)*(s-50))*(s>=50)-30;
    vertY = vertY + (sin(theta)*s*(s<50))+(sin(theta_base)*50+sin(theta)*(s-50))*(s>=50);
    domain = TargetDomain(vertX, vertY);
end

function domain = linearMotion(vertX,vertY,s,tl,vDomain)
    domain = TargetDomain(vertX+(s-tl)*vDomain, (vertY+(s-tl)*vDomain)^2);
end

function domain = circleMotion(vertX,vertY,s,vDomain)
    rotMat = [cos(s*vDomain) -sin(s*vDomain); sin(s*vDomain) cos(s*vDomain)];
    oldVertices = [vertX;vertY];
    rotVertices = rotMat * oldVertices;
    domain = TargetDomain(rotVertices(1,:),rotVertices(2,:));
end