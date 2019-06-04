classdef TFM < Node
  % Traffic flow manager
  % Checks safety of all vehicles
  % Stores necessary reachable sets for safety and liveness
  % Stores global constants in the air space
  
  properties
    % Highways
    hws = {};
    hw_speed = 10;
    
    % Separation distance of vehicles within platoons
    ipsd = 20; % Intra-platoon separation distance
    
    % active agents
    aas = {};
    
    % safety time
    safetyTime = 2;
    
    % collision radius
    cr = 5;
    
    % Thresholds for being considered in target set or in reachable set
    ttt = 1; % target threshold time
    rtt = 12; % reachable set threshold time
    
    % Frequency of state updates to the system
    dt = 0.1;
    
    %% Quadrotor reachable sets    
    % Quadrotor-quadrotor safety reachable set
    qr_qr_safe_V
       
  end
  
  % No explicit constructor
end % end classdef