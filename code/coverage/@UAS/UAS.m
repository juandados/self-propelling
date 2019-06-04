classdef UAS < Node
  properties
    % Unmanned aerial system class. Super class of UTM vehicles
    ID          % ID number (global, unique)
    
    % Mode
    %   'Free'
    %   'Follower'
    %   'Leader'
    %   'Faulty'
    q = 'Free'
    
    % Status (when requesting control from TFM)
    %   'idle'
    %   'busy'
    tfm_status = 'idle'
    
    %% Platoon-related properties
    p           % Pointer to platoon
    idx         % Vehicle index in platoon (determines phantom position)
    FQ          % Pointer to quadrotor in front (self if leader)
    BQ          % Pointer to quadrotor behind (self if tail)
    
    pJoin       % platoon that vehicle is trying to join
    
    h_abs_target_V     % for getting to an absolute target
    h_rel_target_V     % for getting to a relative target
    h_safe_V           % Safety sets
    h_safe_V_list      % List of vehicles for which safety set is being plotted
    
    % No constructor in DynSys class. Use constructors in the subclasses
  end
end