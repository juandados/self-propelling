clear all;
uMax = 3;
uMin = -3;
x = [uMin:0.01:uMax];
X = 2*x;
nx = numel(x);
y = [uMin:0.01:uMax];
Y = 2*y;
ny = numel(y);
vx = -10;
vy = -10;
% vx * x + vy * y - B = 0 implies y = -vx/vy * x + B
M = -vx/vy;
C = -27.75;
B = -C/vy;
l = @(x,y) vx*x + vy*y + C;
% optimal controller
uxo = (vx>=0)*uMax + (vx<0)*uMin;
uyo = (vy>=0)*uMax + (vy<0)*uMin;
% goal controller
uxg = (uMax-uMin)*rand + uMin;
uyg = (uMax-uMin)*rand + uMin;

% feasible controller:
h = l(uxg, uyg);
if h > 0
    uxf = uxg;
    uyf = uyg;
else
    % finding line - square intercepts
    signs = [l(uMin, uMin)>0, l(uMin, uMax)>0, ...
        l(uMax, uMax)>0, l(uMax, uMin)>0, l(uMin, uMin)>0];
    cutIndicator = abs(diff(signs)) == 1;
    Ax = [vx, vy; 1, 0];
    Ay = [vx, vy; 0, 1];
    Cm = [-C, -C; uMin, uMax];
    intx = inv(Ax)*Cm;
    inty = inv(Ay)*Cm;
    int = [intx(:,1)'; inty(:,2)'; intx(:,2)'; inty(:,1)'];
    candidates = int(cutIndicator,:);
    d = candidates(1,:)-[uxg, uyg];
    D = candidates(1,:) - candidates(2,:);
    uf = candidates(1,:) - (D * d' * D / norm(D)^2);
    if (uf(:,1)>uMax | uf(:,1)<uMin | uf(:,2)>uMax | uf(:,2)<uMin)
        normsToCandidates = apply(@(x)norm(x),(candidates - repmat([uxg, uyg], 2, 1)));
        uf = candidates(normsToCandidates == min(normsToCandidates),:);
    end
    uxf = uf(1); uyf = uf(2);
end

signs = [l(uMin, uMin)>0, l(uMin, uMax)>0, ...
        l(uMax, uMax)>0, l(uMax, uMin)>0, l(uMin, uMin)>0];

% plotting
fig = figure(1); clf(fig); hold on;
% plotting controller domain
plot(x, repmat(uMin, ny));
plot(x, repmat(uMax, ny));
plot(repmat(uMin,nx), y);
plot(repmat(uMax,nx), y);
% plotting line min gradV.f=0;
plot(X,M*X+B);
% plotting controllers
plot(uxo, uyo, 'ro');
plot(uxg, uyg, 'go');
plot(uxf, uyf, 'yo');
% axis settings
axis([uMin, uMax, uMin, uMax]*2);
axis square;
