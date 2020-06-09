function [safe, uSafeOptimal] = checkPWSafety(obj, i, j)
% function checkPWSafety(obj, i, j)
%          (check pairwise safety)
%
% Checks the safety of active agent i with respect to active agent j, and
% gives a safety-preserving input if necessary
%
% Inputs:  obj - tfm object
%          i   - index of agent for whom safety is checked (evader)
%          j   - index of agent against whom safety is checked (pursuer)
% Outputs: safe - boolean specifying whether agent i is safe with respect
%                 to agent j
%          uSafe - safety-preserving input (empty if not needed)
%
% Juan Chacon, 2020-jun-03

% Vehicle is safe with respect to itself
if isempty(obj.aas{j}) || obj.aas{i} == obj.aas{j}
  safe = 1;
  uSafeOptimal = [];
  return
end

switch(class(obj.aas{i}))
  case 'UTMDubinsCarAccelerated'
    switch(class(obj.aas{j}))
      case 'UTMDubinsCarAccelerated'
        % Checking if collition
        distancePW = norm(obj.aas{i}.getPosition - obj.aas{j}.getPosition);
        if distancePW < obj.cr
            disp(['Collision!!!: Pair i:', num2str(i), ' j:', num2str(j)]);
            flag = 1;
        else
            flag = 0;
        end
        try
            obj.collisions{i,j} = [obj.collisions{i,j},flag];
        catch
            obj.collisions{i,j} = [flag];
        end
        % Getting controllers
        [safe,  uSafeOptimal] = checkPWSafety_qr_qr(obj.qr_qr_safe_V, obj.safetyTime, obj.aas{i}, obj.aas{j});
        if ~safe
            disp(['Unsafe!!!: A pair i:', num2str(i), ' j:', num2str(j), ' is unsafe']);
            disp(['Optimal Controller: ', num2str(uSafeOptimal')]);
            obj.unsafeCount = obj.unsafeCount + 1;
        end
    otherwise
        error('Unknown agent type')
        
    end % end inner switch
  otherwise
    error('Unknown agent type')
end % end outer switch

end % end function

%%
function [safe, uSafeOptimal] = checkPWSafety_qr_qr(qr_qr_safe_V, safetyTime, evader, pursuer)
% Safety of quadrotor qr1 with respect to quadrotor qr2

% Heading of evader
theta1 = evader.getHeading;
theta2 = pursuer.getHeading;
pos1 = evader.getPosition;
pos2 = pursuer.getPosition;
speed1 = evader.getSpeed;
speed2 = pursuer.getSpeed;
base_x = [cos(theta1)*(pos2(1)-pos1(1)) + sin(theta1)*(pos2(2)-pos1(2));
    -sin(theta1)*(pos2(1)-pos1(1)) + cos(theta1)*(pos2(2)-pos1(2));
    theta2 - theta1;
    speed1;
    speed2];

% Juan: Is the rotation necessary?

% Compute safety value
valuex = eval_u(qr_qr_safe_V.g, qr_qr_safe_V.data, base_x);

% Compute safety preserving control if needed
if (valuex > 0) || any(qr_qr_safe_V.g.max([1,2]) < base_x([1,2])) ...
        || any(qr_qr_safe_V.g.min([1,2]) > base_x([1,2]))
  % Safe; no need to worry about computing controller
  safe = 1;
  uSafeOptimal = [];
  return
end

% Not safe, compute safety controller
safe = 0;

%juan: CalculateCostate is not the proper function to call....
base_grad = calculateCostate(qr_qr_safe_V.g, qr_qr_safe_V.grad, base_x);

% Controller2: |u|_{2}<u_{max} optimal
u_theta = sign(base_grad(1)*base_x(2) - base_grad(2)*base_x(1) - base_grad(3)) * evader.wMax;
u_v = sign(base_grad(4))*evader.aMax;
u = [u_theta; u_v];
% Rotate the control to correspond with the actual heading of the "pursuer"
% Optimal controller is the one that maximize the hamiltonian
%uSafeOptimal = rotate2D(u, theta);
uSafeOptimal = u;
end

function base_grad = calculateCostate(g, grad, base_x)
    base_grad = zeros(numel(grad),1);
    for i = 1:numel(grad)
        base_grad(i) = eval_u(g, grad{i}, base_x);
    end
end