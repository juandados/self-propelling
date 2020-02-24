classdef UTMDubinsCarAccelerated < DubinsCarAccelerated & UAS
  % Note: Since quadrotor is a "handle class", we can pass on
  % handles/pointers to other quadrotor objects
  % e.g. a.platoon.leader = b (passes b by reference, does not create a copy)
  % Also see constructor
  
  properties
    
    % Waypoints to follow;
    % Warning 1: For now this only comes up in getToPose, but eventually,
    % it may be a good idea to always use this.
    % Warning 2: Right now it's always a linear function; however, ideally
    % it should just be a few points
    waypoints = []
    
    A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0]
    
    B = [0 0; 1 0; 0 0; 0 1]
  end % end properties
  
  methods
    function obj = UTMDubinsCarAccelerated(varargin)
      % obj = UTMQuad4D(x, uMax)
      obj@DubinsCarAccelerated(varargin{:});
    end % end constructor
  end % end methods
end % end class
