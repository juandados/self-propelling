clear all;
close all;
%rng(6);
u = randn(1,2);
u = 2*u/norm(u);
%u = [43.8523   11.2120];

uThetaMax = pi/10;
uVMax = 5;
box = [];
theta = pi/6;
v = 2;
%theta = 2.7394;
%v = 0.0085;

R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
lims = [uVMax, v*uThetaMax]';
verts = diag(lims) * [1 1 -1 -1 1;-1 1 1 -1 -1];
verts2 = R * verts;
figure();
hold on;
plot(verts(1,:),verts(2,:),'b.--');
plot(verts2(1,:),verts2(2,:), 'r.--');
% projection
ur = inv(R)*u';
absUr = abs(ur);
mins = min(absUr, lims);
cutter_dim = (absUr(2)/absUr(1) > lims(2)/lims(1))+1;
cutter_dim = cutter_dim(1);
t = mins(cutter_dim)/absUr(cutter_dim);
ur_proj = ur*t;
u_proj =  R*ur_proj;
%urThresholded = max(ur,[uVmax, uThetaMax])
plot([0,u(1)],[0,u(2)],'r');
plot([0,u_proj(1)],[0,u_proj(2)],'k');
plot([0,ur(1)],[0,ur(2)],'b');
plot([0,ur_proj(1)],[0,ur_proj(2)],'g');

axis equal

%% function definition
