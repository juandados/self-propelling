classdef UTMPlane4D < Plane4D & UAS
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
    speedLimit;
    speedMin;
  end % end properties
  
  methods
    function obj = UTMPlane4D(varargin)
      
      if nargin < 7
        speedMin = 0;
      else
      	speedMin = varargin{7};
        varargin(:,7)=[];
      end
      
      if nargin < 6
        speedLimit = 10;
      else
        speedLimit = varargin{6};
        varargin(:,6)=[];
      end
      
      % obj = UTMQuad4D(x, uMax)
      obj@Plane4D(varargin{:});
    end % end constructor
  end % end methods
end % end class
