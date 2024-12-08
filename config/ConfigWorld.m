classdef ConfigWorld < Configuration

  %% Properties
  properties (SetAccess = {?ConfigWorld, ?Configuration}, GetAccess = public)
    % General Settings
    time_step (1, 1) double = 0.001;  % [s]
    max_simulation_time (1, 1) double = 4.0;  % [s]
    use_dynamics (1, 1) logical = true;  % true/false
    gravity (1, 1) double = 1 / 6;  % [G]

    % Simulation Termination Settings
    sim_stop_time_max (1, 1) logical = true;
  end

  %% Constructor
  methods (Access = public)

    function config_world = ConfigWorld(config)
    % ConfigWorld() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_world = config_world.override(config);
    end

  end

end  % ConfigWorld
