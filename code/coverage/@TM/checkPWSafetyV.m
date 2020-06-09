function [safe, uSafeOptimal, safe_val] = checkPWSafetyV(obj, i, j)
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
% Mo Chen, 2015-11-04
% Modified: Mo Chen, 2015-12-13

% Vehicle is safe with respect to itself
if isempty(obj.aas{j}) || obj.aas{i} == obj.aas{j}
  safe = 1;
  uSafeOptimal = [];
  uSafeLeft = [];
  uSafeRight = [];
  safe_val = 10;
  return
end

switch(class(obj.aas{i}))
  case 'UTMQuad4D'
    %% agent i is a quadrotor
    switch(class(obj.aas{j}))
      case 'UTMQuad4D'
        % Checking if collition
        distancePW = norm(obj.aas{i}.getPosition - obj.aas{j}.getPosition);
        if distancePW < obj.cr
            disp(['Collision!!!: Pair i:', num2str(i), ' j:', num2str(j)]);
            %disp(['Collision!!!: Pair i:', num2str(i), ' j:', num2str(j), ...
            %    ': (', num2str(obj.aas{i}.x'), '), (', num2str(obj.aas{j}.x'), ')']);
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
        [safe,  uSafeOptimal, safe_val] = checkPWSafety_qr_qr(obj.qr_qr_safe_V, obj.aas{i}, obj.aas{j});
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
function [safe, uSafeOptimal, valuex] = checkPWSafety_qr_qr(qr_qr_safe_V, evader, pursuer)
% Safety of quadrotor qr1 with respect to quadrotor qr2

% Heading of evader
theta = evader.getHeading;

% Get relative state assuming pursuer faces 0 degrees
base_pos = rotate2D(evader.getPosition - pursuer.getPosition, -theta);
base_vel = rotate2D(evader.getVelocity - pursuer.getVelocity, -theta);
base_x = [base_pos(1); base_vel(1); base_pos(2); base_vel(2)];

% Compute safety value
valuex = eval_u(qr_qr_safe_V.g, qr_qr_safe_V.data, base_x);

% Compute safety preserving control if needed
if (valuex > 0) || any(qr_qr_safe_V.g.max < base_x) || any(qr_qr_safe_V.g.min > base_x)
  % Safe; no need to worry about computing controller
  safe = 1;
  uSafeOptimal = [];
  uSafeLeft = [];
  uSafeRight = [];
  valuex = 10;
  return
end

% Not safe, compute safety controller
safe = 0;

%juan:
base_grad = calculateCostate(qr_qr_safe_V.g, qr_qr_safe_V.grad, base_x);

% Controller1: |u|_{inf}<u_{max}
%ux = (base_grad(2)>=0)*evader.uMax + (base_grad(2)<0)*evader.uMin;
%uy = (base_grad(4)>=0)*evader.uMax + (base_grad(4)<0)*evader.uMin;

% Controller2: |u|_{2}<u_{max} optimal
normalizer = norm([base_grad(2), base_grad(4)]) + eps;
ux = (base_grad(2))*evader.uMax/normalizer;
uy = (base_grad(4))*evader.uMax/normalizer;
uo = [ux; uy];

% Rotate the control to correspond with the actual heading of the "pursuer"
% Optimal controller is the one that maximize the hamiltonian
uSafeOptimal = rotate2D(uo, theta);
disp(['base_x: ', num2str(base_x'), ' valuex:', num2str(valuex')]);
end

function base_grad = calculateCostate(g, grad, base_x)
    base_grad = zeros(numel(grad),1);
    for i = 1:numel(grad)
        base_grad(i) = eval_u(g, grad{i}, base_x);
    end
end