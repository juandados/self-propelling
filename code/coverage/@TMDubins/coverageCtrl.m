function uCoverage = coverageCtrl(obj)
% u = joinPlatoon(obj, platoon)
% method of Quadrotor class
%
% Requests the control signal to join a platoon from the tfm
vertX = obj.domain.vertX;
vertY = obj.domain.vertY;

n = length(obj.aas);
rd = sqrt(obj.domain.area/n);

aas = [obj.aas{:}];
states = [aas.x];
% Juan: This changed as the position states are in coords 1 and 2
X = states([1,2],:)';
% Juan = This transformation may be not necessary as the controller depends
% only on the norm of V with is in the coordinate 4
V = [states([4],:).*cos(states([3],:)); states([4],:).*sin(states([3],:))]';

delta = 0;
b = 1+delta;
fI = @(r)0.2*1*(r-rd).*(r-(rd*b)<0);
b = 1+delta/2;
fh = @(h)0.2*2*(h+rd/2).*(h-(-b*rd/2)>0);

l_al = 1;
c_al = 3;
dV2 = zeros(size(V));
for i = 1:n
    % Vehicle-Vehicle
    R = ones(n,1)*X(i,:) - X;
    r = apply(@(v)norm(v,2), R);
    DUsI = (fI(r)./r)*ones(1,2).*R;
    DUsI(i,:) = zeros(1,2);
    % Velocity-alignment
    Rv = ones(n,1)*V(i,:) - V;
    DUsV = c_al*exp(-r/l_al).*Rv;
    % Vehicle-Domain
    [h, x_proj,y_proj] = poly_dist(X(i,1),X(i,2),vertX,vertY);
    H = X(i,:)-[x_proj, y_proj];
    DUsH = repmat(fh(h),1,2).*H/h;
    dV2(i,:) = sum(DUsI,1) + sum(DUsH,1) + sum(DUsV,1);
end

vd = 0.3 * sqrt(2); %a = 0.15;
a = 1* 0.3;
v = apply(@(v)norm(v,2), V);
dV1 = -a*((v-vd)./(v+eps))*ones(1,2).*V;
u = dV1 - dV2;

%Thresholding Rectangular Force and :
if true
    uDubinsCar = obj.projectDubinsForce(u, states);
end


uCoverage = cell(n, 1);
for i=1:n
    uCoverage{i} = uDubinsCar(i,:);
end

end
