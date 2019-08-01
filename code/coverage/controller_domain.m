t = [0:0.05:2*pi];
x = cos(t);
y = sin(t);
m = -4/9;
b1 = 0.5;
b2 = 0.9;
xl=[-1:0.1:1];
yl1 = m * xl + b1;
yl2 = m * xl + b2;
hold on;
plot(xl,yl1);
plot(xl,yl2);
plot(x,y);
axis([-1,1,-1,1]);

%% 
gradv = [1,1,2,-2];
vxr = 0.5;
vyr = -1;
uxj = -1;
uyj = 1;
umax = 3;
t = [0:0.05:2*pi];
x = umax*cos(t);
y = umax*sin(t);
M = -gradv(2)/gradv(4);
B = (-gradv(1)*vxr+gradv(2)*uxj-gradv(3)*vyr+gradv(4)*uyj)/gradv(4);
xl= [-umax:0.1:umax];
yl = M * xl + B;
%optimal controller:
c1 = [gradv(2),gradv(4)]*umax/norm([gradv(2),gradv(4)]);
% intercepts line circle:
a = 1+(gradv(2)/gradv(4))^2;
b = -2*(gradv(2)/gradv(4)^2)*(-gradv(1)*vxr+gradv(2)*uxj-gradv(3)*vyr+gradv(4)*uyj);
c = ((-gradv(1)*vxr+gradv(2)*uxj-gradv(3)*vyr+gradv(4)*uyj)/gradv(4))^2-umax^2;
M = -gradv(2)/gradv(4);
B = (-gradv(1)*vxr+gradv(2)*uxj-gradv(3)*vyr+gradv(4)*uyj)/gradv(4);
x2 = (-b + sqrt(b^2-4*a*c))/(2*a);
x3 = (-b - sqrt(b^2-4*a*c))/(2*a);
c2 = [x2, M * x2 + B];
c3 = [x3, M * x3 + B];
%
figure(1);
hold on;
plot(xl,yl);
plot(x,y);
plot(c1(1),c1(2),'*');
plot(c2(1),c2(2),'*');
plot(c3(1),c3(2),'*');
axis([-umax,umax,-umax,umax]);
axis square;
%% intercepts line circle after realize the minimum value for disturbance
% gradv(2)*uxj + gradv(4)*uyj is -umax^2:
gradv = [1,1,2,-2];
vxr = 0.7;
vyr = 0.3;
umax = 3;
t = [0:0.05:2*pi];
x = umax*cos(t);
y = umax*sin(t);
M = -gradv(2)/gradv(4);
B = (-gradv(1)*vxr-gradv(3)*vyr+norm([gradv(2),gradv(4)])*umax)/gradv(4);
xl= [-umax:0.1:umax];
yl = M * xl + B;
a = 1+(gradv(2)/gradv(4))^2;
b = -2*(gradv(2)/gradv(4)^2)*(-gradv(1)*vxr-gradv(3)*vyr+norm([gradv(2),gradv(4)])*umax);
c = ((-gradv(1)*vxr-gradv(3)*vyr+norm([gradv(2),gradv(4)])*umax)/gradv(4))^2-umax^2;
%optimal controller:
c1 = [gradv(2),gradv(4)]*umax/norm([gradv(2),gradv(4)]);

x2 = (-b + sqrt(b^2-4*a*c))/(2*a);
x3 = (-b - sqrt(b^2-4*a*c))/(2*a);
c2 = [x2, M * x2 + B];
c3 = [x3, M * x3 + B];
%
figure(1);
hold on;
plot(xl,yl);
plot(x,y);
plot(c1(1),c1(2),'*');
plot(c2(1),c2(2),'*');
plot(c3(1),c3(2),'*');
axis square;
