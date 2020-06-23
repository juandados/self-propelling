function uCoverage = coverageCtrl(obj)
% u = joinPlatoon(obj, platoon)
% method of Quadrotor class
%
% Requests the control signal to join a platoon from the tfm
vertX = obj.domain.vertX;
vertY = obj.domain.vertY;

N = length(obj.aas);
rd = sqrt(obj.domain.area/N);

aas = [obj.aas{:}];
states = [aas.x];
X = states([1,3],:)';
V = states([2,4],:)';

delta = 0;
b = 1+delta;
fI = @(r)obj.a_I*(r-rd).*(r-(rd*b)<0);
b = 1+delta/2;
fh = @(h)obj.a_h*(h+rd/2).*(h-(-b*rd/2)>0);

%l_al = -rd/log(0.1);
l_al = -rd/log(obj.l_al_decay);

dV2 = zeros(size(V));
for i = 1:N
    R = ones(N,1)*X(i,:) - X;
    r = apply(@(v)norm(v,2), R);
    DUsI = (fI(r)./r)*ones(1,2).*R;
    DUsI(i,:) = zeros(1,2);
    % Velocity-alignment
    Rv = ones(N,1)*V(i,:) - V;
    DUsV = obj.c_al*exp(-r/l_al).*Rv;
    % Vehicle-Domain
    [h, x_proj,y_proj] = poly_dist(X(i,1),X(i,2),vertX,vertY);
    H = X(i,:)-[x_proj, y_proj];
    DUsH = repmat(fh(h),1,2).*H/h;
    dV2(i,:) = sum(DUsI,1) + sum(DUsH,1) + sum(DUsV,1);
end

%vd = 0; %a = 0.15;
%v = apply(@(v)norm(v,2), V);
%dV1 = -obj.a_v*((v-vd)./v)*ones(1,2).*V;
dV1 = -obj.a_v * (V - transpose(obj.vd));
u = dV1 - dV2;

%Thresholding the force:
if true
    acc_max = obj.uMax;
    %acc_max = 3;
    %acc_max = 100;
    u_norm = apply(@(v)norm(v,2), u);
    u = (u./u_norm).*min(acc_max, u_norm);
    %figure(3); hold off; plot(dV_norm, 'r*'); hold on;
    %plot(apply(@(v)norm(v,p), dV), 'b.'); axis([1,N,-5, 100]);
end

uCoverage = cell(N, 1);
for i=1:N
    uCoverage{i} = u(i,:);
end

end
