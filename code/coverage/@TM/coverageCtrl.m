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
X = states([1,3],:)';
V = states([2,4],:)';

delta = 0;
b = 1+delta;
fI = @(r)1*(r-rd).*(r-(rd*b)<0);
b = 1+delta/2;
fh = @(h)2*(h+rd/2).*(h-(-b*rd/2)>0);

dV2 = zeros(size(V));
for i = 1:n
    R = ones(n,1)*X(i,:) - X;
    r = apply(@(v)norm(v,2), R);
    DUsI = (fI(r)./r)*ones(1,2).*R;
    DUsI(i,:) = zeros(1,2);
    [h, x_proj,y_proj] = poly_dist(X(i,1),X(i,2),vertX,vertY);
    H = X(i,:)-[x_proj, y_proj];
    DUsH = repmat(fh(h),1,2).*H/h;
    dV2(i,:) = sum(DUsI,1) + sum(DUsH,1);
end


vd = 0; %a = 0.15;
a = 1*0.2;
v = apply(@(v)norm(v,2), V);
dV1 = -a*((v-vd)./v)*ones(1,2).*V;
u = dV1 - dV2;

%Thresholding the force:
if true
    %acc_max = obj.uMax;
    %acc_max = 3;
    acc_max = 100;
    u_norm = apply(@(v)norm(v,2), u);
    u = (u./u_norm).*min(acc_max, u_norm);
    %figure(3); hold off; plot(dV_norm, 'r*'); hold on;
    %plot(apply(@(v)norm(v,p), dV), 'b.'); axis([1,n,-5, 100]);
end

uCoverage = cell(n, 1);
for i=1:n
    uCoverage{i} = u(i,:);
end

end
