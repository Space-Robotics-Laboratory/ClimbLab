classdef ConfigPerception < Configuration
% Configuration for perception (sensing camera) parameters
%
% Created     : 2021.02.10 by Keigo Haji
% Last updated: 2025.01.06 by Masazumi Imai

  %% Properties
  properties (SetAccess = {?ConfigPerception, ?Configuration}, GetAccess = public)
    kUseSensingCamera_ (1, 1) logical = false;  % true/false

    % Sensing type
    % options: "RealSense_D435i"
    kSensingType_ (1, 1) string = "RealSense_D435i";

    % Mounting pose settings
    kMountingPosition_ (3, 1) double = [0.079; -0.011 ; 0.07];      % [m] (Measured by CAD of Hubrobo v3.2)
    kMountingAngle_    (3, 1) double = deg2rad([0.0; -45.0; 0.0]);  % (euler) [rad] (Measured by CAD of Hubrobo v3.2)

    % Field of View settings
    kFOVHorizontal_  (1, 1) double = deg2rad(86.0);  % [rad] (RealSense D435i)
    kFOVVertical_    (1, 1) double = deg2rad(57.0);  % [rad] (RealSense D435i)
    kFOVMaxDistance_ (1, 1) double = 2.0;   % [m]   (RealSense D435i)
    kFOVMinDistance_ (1, 1) double = 0.28;  % [m]   (RealSense D435i)

    % Initial known area settings
    % options: "circle"
    kInitialKnownAreaShape_ (1, 1) string = "circle";
      % Parameters for "circle"
      kCircularRadiusFromBaseCoM_ (1, 1) double = 0.4;  % [m]
  end

  %% Constructor
  methods (Access = public)

    function config_perception = ConfigPerception(config)
    % Constructor
    % Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_perception = config_perception.override(config);
    end

  end

  %% Getter
  methods (Access = public)
  end

end  % ConfigWorld
