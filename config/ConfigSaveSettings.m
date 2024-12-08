classdef ConfigSaveSettings < Configuration

  %% Properties
  properties (SetAccess = {?ConfigSaveSettings, ?Configuration}, GetAccess = public)
    % Time interval for saving variables (should be larger than time-step)
    variable_saving_time_interval (1, 1) double;
    % Save basic variables to csv file
    save_csv_file (1, 1) logical = true;
    % Save the loaded config file in the dat folder if config is not "default"
    save_config_file (1, 1) logical = true;
  end

  %% Public Methods
  methods (Access = public)

    function config_save_settings = ConfigSaveSettings(config, config_world)
    % ConfigSaveSettings() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
        config_world (1, 1) {mustBeA(config_world, "ConfigWorld")};
      end

      config_save_settings.variable_saving_time_interval = config_world.time_step;

      if (config == "default")
        return;
      end

      config_save_settings = config_save_settings.override(config);
    end

  end

end  % ConfigSaveSettings
