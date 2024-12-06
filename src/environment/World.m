classdef World
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    time_step (1, 1) double;  % [s]
    max_simulation_time (1, 1) double;  % [s]
    use_dynamics (1, 1) logical;
    gravity (1, 1) double;  % [G]
  end

  %% Public Methods
  methods (Access = public)

    function world = World(config)
    % World() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigWorld")};
      end
      global d_time Gravity Ez;

      % Clone properties value from config file
      config_prop_name = properties(config);
      this_prop_name = properties(world);
      for i = 1:length(config_prop_name)
        if (~any(strcmp(this_prop_name, config_prop_name{i, 1})))
          continue;
        end
        world.(config_prop_name{i, 1}) = config.(config_prop_name{i, 1});
      end

      d_time = world.time_step;
      Gravity = world.gravity * [0.0; 0.0; -9.81];
      Ez = [0; 0; 1];
    end

  end

  %% Getter
  methods (Access = public)
    function use_dynamics = getUseDynamics(world)
      use_dynamics = world.use_dynamics;
    end
    function max_simulation_time = getMaxSimulationTime(world)
      max_simulation_time = world.max_simulation_time;
    end
  end

end
% EOF