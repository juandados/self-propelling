function tm = simulateMultiDubins(recordVideo, saveFigures, extraArgs)
% Simulates 2 dubins car 
rng(30);

addpath('..');

if nargin < 1
  recordVideo = false;
end

if nargin < 2
  saveFigures = false;
end

if nargin < 3
  extraArgs = struct();
end

% ---- Setting Extra Parameters ----

if isfield(extraArgs,'N')
    N = extraArgs.N;
else
    N = 6;
end

if isfield(extraArgs,'avoidance')
    avoidance = extraArgs.avoidance;
else
    avoidance = true;
end

if isfield(extraArgs,'domainType')
    domainType = extraArgs.domainType;
else
    domainType = 'triangle';
end

if isfield(extraArgs,'vd')
    vd = extraArgs.vd;
else
    vd = @(t) [0;0];
end

if isfield(extraArgs,'domainPath')
    domainPath = extraArgs.domainPath;
else
    domainPath = @(t) t*vd(t);
end

if isfield(extraArgs,'domainRotationAngle')
    domainRotationAngle = extraArgs.domainRotationAngle;
else
    domainRotationAngle = @(t) 0;
end

if isfield(extraArgs, 'initialConfig')
    initialConfig = extraArgs.initialConfig;
else
    initialConfig = 'arrowPaper';
end

if isfield(extraArgs,'c_al')
    c_al = extraArgs.c_al;
else
    c_al = 0;
end

if isfield(extraArgs,'l_al_decay')
    %l_al_decay is the proportion of c_al that will be applied at r_d, e.g
    %0.5 means at rd the alignment force is 0.5 c_al.
    l_al_decay = extraArgs.l_al_decay;
else
    l_al_decay = 0.5;
end

if isfield(extraArgs, 'a_I')
    a_I = extraArgs.a_I;
else
    a_I = 1;
end

if isfield(extraArgs, 'a_h')
    a_h = extraArgs.a_h;
else
    a_h = 1;
end

if isfield(extraArgs, 'a_v')
    a_v = extraArgs.a_v;
else
    a_v = 1;
end

if isfield(extraArgs, 'tMax')
    tMax = extraArgs.tMax;
else
    tMax = 50;
end

if ~isfield(extraArgs, 'tailSize')
    extraArgs.tailSize = 50;
end
    

screenShotsCount = 50;

% ---- Traffic Manager ----
tm = TMDubins;
% setup speed limit
tm.speedLimit = 9.5;
% Min Speed
tm.speedMin = 0.5;
% collision radius
tm.cr = 2;
% safety time (it will be safe during the next st seconds)
tm.safetyTime = 5;
% maximum force for vehicles
tm.uThetaMax = pi/2; %same as omegaMax or wMax in model
tm.uVMax = 3; %same as aMax in model
% domain desired velocity
tm.vd = vd(0);
% inter vehicle alignement strength
tm.c_al = c_al;
% l_al decay, i.e. proportion of c_al that will be applied at r_d
tm.l_al_decay = l_al_decay;
% inter vehicle non zero slope
tm.a_I = a_I;
% vehicle domain non zero slope
tm.a_h = a_h;
% vehicle domain aligment stregth
tm.a_v = a_v;

% compute reachable set
tm.computeRS('db_db_safe_V_circle');

% domain setup
switch domainType
    case 'circle'
        r = 1/100;
        ns = 100;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = r * 50 * sin(t);
        vertY = r * 50 * cos(t);
    case 'diamond'
        r = 0.1;
        vertX = [-50, 0, 50, 0];
        vertY = [0, 50, 0, -50]*r;
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
        vertX = 5 * sin(t);
        vertY = 5 * cos(t);
    case 'pentagon'
        ns = 5;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t);
        vertY = 50 * cos(t);
    case 'trianglePaper'
        r = 0.3;
        ns = 3;
        t = linspace(0, 2*pi*(1-1/ns), ns);
        vertX = 50 * sin(t)*r;
        vertY = 50 * cos(t)*r;
        vertY = vertY - (max(vertY)+min(vertY))/2;
    case 'squarePaper'
        r = 0.20;
        vertX = [-50,-50, 50, 50]*r;
        vertY = [-50, 50, 50, -50]*r;
    case 'arrowPaper'
        r = 0.3;
        vertX = r*[50, -50, 0, 0];
        vertY = r*[50, 0, 0, -50];
        %safetyTime = 5;
        %tm.speedLimit = 10;
        %N=9;
        %extraArgs.tailSize = -1;
end

domain = TargetDomain(vertX, vertY);
tm.addDomain(domain);

% figure setup
f = figure;
domain.domainPlot('blue', 'red');
hold on;
f.Children.FontSize = 16;
f.Position(1:2) = [100 100];
f.Position(3:4) = [750 750];
f.Color = 'white';
scale = 10;
xlim([min([vertX,vertY]) max([vertX,vertY])]*scale);
ylim([min([vertX,vertY]) max([vertX,vertY])]*scale);

title('t=0');
axis square;

% ---- Adding Dubins Cars ----
if strcmp(initialConfig,'line')
    px = linspace(-30,30,N)';
    py = -15*ones(N,1);
elseif strcmp(initialConfig, 'arrowPaper')
    xx = linspace(-10,10,N);
    px = xx-10;
    py = -1*xx-10;
elseif strcmp(initialConfig, 'random')
    px = -10 * rand(N,1);
    py = -10 * rand(N,1);
elseif strcmp(initialConfig, 'square')
    s = 1;
    px = 15*(mod(0:15,4)/3-0.5)+s*rand(1,16);
    py = 15*(floor([0:15]/4)/3-0.5)+s*rand(1,16);
end

theta = 2 * pi * rand(N,1);
v = tm.speedMin*[ones(N,1), zeros(N,1)];

% registering vehicles in traffic manager
wMax = tm.uThetaMax;
aMax = tm.uVMax;
for j = 1:length(px)
  q = UTMPlane4D([px(j) py(j) theta(j) v(j)], wMax, aMax, 0, [1:4], tm.speedLimit, tm.speedMin);
  %q = UTMDubinsCarAccelerated([px(j) py(j) theta(j) v(j)], wMax, aMax, tm.speedLimit, tm.speedMin);
  tm.regVehicle(q);
end

% plotting initial conditions
colors = lines(length(tm.aas));
for j = 1:length(tm.aas)
  extraArgs.Color = colors(j,:);
  extraArgs.ArrowLength = 3; %j prev 1
  extraArgs.LineStyle = 'none';
  extraArgs.LineWidth = 0.1; %j
  tm.aas{j}.plotPosition(extraArgs);
end

drawnow

% Time integration
tm.dt = 0.1;
t = 0:tm.dt:tMax;

% setting up figure saving
if saveFigures
%     directory = ['figuresDubins/', domainType,' N',num2str(N),' ',datestr(datetime('now'))];
    directory = ['figuresDubins/', domainType,' N',num2str(N),' ', now];
    mkdir(directory);
    %saving meta data
    fileID = fopen([directory,'/metadata.txt'],'w');
    fprintf(fileID, 'number of vehicles: %d\n', N);
    fprintf(fileID, 'avoidance included: %s\n', num2str(avoidance));
    fprintf(fileID, 'domain type: %s\n', domainType);
    fprintf(fileID, 'domain velocity: %s\n', char(vd));
    fprintf(fileID, 'domain path: %s\n', char(domainPath));
    fprintf(fileID, 'domain rotation angle: %s\n', char(domainRotationAngle));
    fprintf(fileID, 'vehicles initial configuration: %s\n', initialConfig);
    fprintf(fileID, 'inter vehicle velocity alignement strength: %f\n', c_al);
    fprintf(fileID, 'inter vehicle nonzero slope: %f\n', a_I);
    fprintf(fileID, 'vehicle domain nonzero slope: %f\n', a_h);
    fprintf(fileID, 'domain velocity strength a_v: %f\n', a_v);
    fprintf(fileID, 'simulation time: %f\n', tMax);
    fprintf(fileID, 'speedLimit: %f\n', tm.speedLimit);
    fprintf(fileID, 'collision radius: %f\n', tm.cr);
    fprintf(fileID, 'safety time: %f\n', tm.safetyTime);
    fprintf(fileID, 'u theta max: %f\n', tm.uThetaMax);
    fprintf(fileID, 'u vel max: %f\n', tm.uVMax);
    fprintf(fileID, 'rd: %f\n', sqrt(tm.domain.area/N));
    fprintf(fileID, 'l_al: %f\n', -sqrt(tm.domain.area/N)/log(l_al_decay));
end

% setting up video recorder
if recordVideo
    vidObj = VideoWriter([directory, '/movie.avi']);
    vidObj.Quality = 100;
    vidObj.FrameRate = 10;
    open(vidObj);
    ax = gca();
end


u = cell(size(tm.aas));
for i = 1:length(t)
  %update domain if  domaing
  vert = [vertX;vertY];
  theta_d = domainRotationAngle(t(i));
  vert = [cos(theta_d) -sin(theta_d); sin(theta_d) cos(theta_d)] * vert + domainPath(t(i));
  domain = TargetDomain(vert(1,:), vert(2,:));
  cla;
  domain.domainPlot('blue', 'red');
  f.Color = 'white';
  axis square
  tm.addDomain(domain);
  tm.vd = vd(t(i));
  disp(['time: ', num2str(t(i))])
  [safe, uSafeOptimal] = tm.checkAASafety;
  uCoverage = tm.coverageCtrl;
        
  for j = 1:length(tm.aas)
    if safe(j) || ~avoidance
      u{j} = uCoverage{j};
    else
      u{j} = uSafeOptimal{j};
    end
    tm.aas{j}.updateState(u{j}, tm.dt);
    extraArgs.Color = colors(j,:);
    tm.aas{j}.plotPosition(extraArgs);
  end
  title(['t=' num2str(t(i))])
  drawnow;
  if recordVideo
      writeVideo(vidObj, getframe(ax));
  end
  if saveFigures && (mod(i-1,floor(length(t)/screenShotsCount))==0 || ismember(t(i),[9,39,70]))
      savefig([directory,'/',num2str(t(i)),'.fig'])
  end
end
%collision count
collisionCount = 0;
for i=1:N
    for j=i+1:N
        collisionCount = collisionCount + sum(abs(diff(tm.collisions{i,j})))/2;
    end
end

disp(['collision count: ', num2str(collisionCount)]);
if saveFigures
    fprintf(fileID, 'collision count: %d\n', collisionCount);
    fclose(fileID);
end
%disp(['unsafe count: ', num2str(tm.unsafeCount)]);
end