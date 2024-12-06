classdef ConfigWorld
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    % General Settings
    time_step (1, 1) double = 0.001;  % [s]
    max_simulation_time (1, 1) double = 4.0;  % [s]
    use_dynamics (1, 1) logical = true;  % true/false
    gravity (1, 1) double = 1/6;  % [G]

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

      config_file_name = "config_" + config;
      if (~isfile("config\preset\" + config_file_name + ".m"))
        error("ERROR: The specified config file does NOT exist.");
      end

      config_file = str2func(config_file_name);
      user_config = feval(config_file);

      this_config_prop_name = properties(config_world);

      meta_class = metaclass(user_config);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigWorld"))
          user_config_prop_name = meta_props(i, 1).Name;

          if (~any(strcmp(this_config_prop_name, user_config_prop_name)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + user_config_prop_name + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          config_world.(user_config_prop_name) = user_config.(user_config_prop_name);
        end
      end
    end

  end

end
% EOF