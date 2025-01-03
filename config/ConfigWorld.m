classdef ConfigWorld < Configuration
% Configuration for world environment
%
% Created     : 2020.07.08 by Warley Ribeiro
% Last updated: 2025.01.03 by Masazumi Imai

  %% Properties
  properties (SetAccess = {?ConfigWorld, ?Configuration}, GetAccess = public)
    % General Settings
    kTimeStep_ (1, 1) double = 0.001;  % [s]
    kMaxSimulationTime_ (1, 1) double = 8.0;  % [s]

    KUseDynamics_ (1, 1) logical = true;  % true/false
    kGravity_ (1, 1) double = 1 / 6;  % [G]

    % Simulation Termination Settings
  end

  %% Constructor
  methods (Access = public)

    function config_world = ConfigWorld(config)
    % Constructor
    % Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_world = config_world.override(config);
    end

  end

  %% Getter
  methods (Access = public)

    function time_step = getTimeStep(config_world)
      time_step = config_world.kTimeStep_;
    end

  end

end  % ConfigWorld
