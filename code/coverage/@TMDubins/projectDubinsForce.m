function u_proj = projectDubinsForce(obj, u, states)
% function projectForce(obj, )
%
% Project Force for all the vehicles
%
% Juan Chacon 2020-10-02

% If adding an empty set, do nothing
if isempty(u)
  return
end

u_proj = zeros(size(u));

for i = 1:size(states,2)
  theta = states(3,i);
  v = abs(states(4,i));%juan: trying to fix the reverse issue add an abs(v) instead of v
  % Rotation Matrix
  R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
  lims = [obj.uVMax, v*obj.uThetaMax;]';
  % Rectangular force projection
  ur = inv(R)*u(i,:)';
  absUr = abs(ur);
  mins = min(absUr, lims);
  cutter_dim = (absUr(2)/absUr(1) > lims(2)/lims(1))+1;
  cutter_dim = cutter_dim(1);
  t = mins(cutter_dim)/absUr(cutter_dim);
  ur_proj_rec = ur*t;
  u_proj_rec(i,:) =  transpose(R*ur_proj_rec);
  % Change force Coordinates
  u_proj(i,:) = u_proj_rec(i,:)*[ cos(theta) -sin(theta)/(v+eps) ; sin(theta) cos(theta)/(v+eps) ];
  u_proj(i,[2,1]) = u_proj(i,[1,2]);
  % checking correctness of the formula
  %u_proj_rec_2 = R*diag([1,v])*u_proj(i,:)';
end
%figure(2);
%u_proj = 0.5*[-1 0; 1 0];
%quiver([0;0],[0;0],u_proj_rec(:,1),u_proj_rec(:,2),'red')
%quiver([0;0],[0;0],u(:,1),u(:,2))
end
