classdef World < handle
% World (environment) parameters
%
% Created     : 2020.04.09 by Warley Ribeiro
% Last updated: 2025.01.03 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kTimeStep_ (1, 1) double;  % [s]
    kMaxSimulationTime_ (1, 1) double;  % [s]
    KUseDynamics_ (1, 1) logical;
    kGravity (1, 1) double;  % [G]
    kGravityVector_ (3, 1) double;  % [m/s^2]
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

      world.kGravityVector_ = world.kGravity * [0.0; 0.0; -9.81];

      d_time = world.kTimeStep_;
      Gravity = world.kGravityVector_;
      Ez = [0; 0; 1];
    end

  end

  %% Getter
  methods (Access = public)

    function max_simulation_time = getMaxSimulationTime(world)
      max_simulation_time = world.kMaxSimulationTime_;
    end

    function use_dynamics = getUseDynamics(world)
      use_dynamics = world.KUseDynamics_;
    end

    function gravity_vector = getGravityVector(world)
      gravity_vector = world.kGravityVector_;
    end

  end

end  % World
