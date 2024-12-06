classdef ConfigSaveSettings
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    % Time interval for saving variables (should be larger than time-step)
    variable_saving_time_interval (1, 1) double;
    % Save basic variables to csv file
    save_csv_file (1, 1) logical = true;
    % Save the loaded config file in the dat folder if config is not "default"
    save_config_file (1, 1) logical = true;
  end

  %% Public Methods
  methods (Access = public)

    function save_settings = ConfigSaveSettings(config, config_world)
    % ConfigSaveSettings() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
        config_world (1, 1) {mustBeA(config_world, "ConfigWorld")};
      end
      save_settings.variable_saving_time_interval = config_world.time_step;

      if (config == "default")
        return;
      end

      config_file_name = "config_" + config;
      if (~isfile("config\preset\" + config_file_name + ".m"))
        error("ERROR: The specified config file does NOT exist.");
      end

      config_file = str2func(config_file_name);
      user_config = feval(config_file);

      this_config_prop_name = properties(save_settings);

      meta_class = metaclass(user_config);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigSaveSettings"))
          user_config_prop_name = meta_props(i, 1).Name;

          if (~any(strcmp(this_config_prop_name, user_config_prop_name)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + user_config_prop_name + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          save_settings.(user_config_prop_name) = user_config.(user_config_prop_name);
        end
      end

    end

  end

end
% EOF