function u = coverDomain(obj, tfm)
% u = joinPlatoon(obj, platoon)
% method of Quadrotor class
%
% Requests the control signal to join a platoon from the tfm
vertX = tfm.domain.vertX;
vertY = tfm.domain.vertY;

n = length(tfm.aas);
rd = sqrt(tfm.domain.area/n);
fI = @(r)10*(r-rd).*(r-rd<0);
fh = @(h)10*(h+rd/2).*(h+rd/2>0);

aas = [tfm.aas{:}];
X = [aas.x];
X = X([1,3],:)';

% Force due distance intra vehicles
R = ones(n,1)*obj.x([1,3])' - X;
r = apply(@(v)norm(v,2), R);
DUsI = (fI(r)./r)*ones(1,2).*R;
I = r == 0;
DUsI(I,:) = [0, 0];
u_vehs = sum(DUsI,1);

% Force due distance to domain
[h, x_poly, y_poly] = poly_dist(obj.x(1), obj.x(3), vertX, vertY);
H = obj.x([1,3])-[x_poly, y_poly];
u_dom = fh(h).*H/h;

% Force due vehicle velocity
V = obj.x([2,4]);
v = norm(V);
vd = 0; a = 0.1;
uv = -a*((v-vd)./v).*V;

% total control signal
%u = -u_dom - u_vehs + uv;
u = -u_dom + u_vehs + uv;
end
