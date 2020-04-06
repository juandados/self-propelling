classdef TMDubins < Node
  % Traffic manager
  % Checks safety of all vehicles
  % Stores necessary reachable sets for safety
  % Stores global constants in the air space
  
  properties
    % Map
    domain;
    
    % Speed Limit
    speedLimit = 10;
    
    % Min Speed
    speedMin = 1;
    
    % active agents
    aas = {};
    
    % safety time
    safetyTime = 2;
    
    % collision radius
    cr = 5;
    
    % Frequency of state updates to the system
    dt = 0.1;
    
    % Juan: maximum forces allowed for vehicles registred in the tm:
    uThetaMax = pi/10;
    uVMax = 3;

    % Juan: varible for collision count
    collisions = {};
    unsafeCount = 0;
    
    %% Quadrotor reachable sets    
    % Quadrotor-quadrotor safety reachable set
    % juan: this should be called qr_qr_safe_T instead of ..._safe_V
    qr_qr_safe_V
       
  end
  
  % No explicit constructor
end % end classdef
