classdef ConfigSaveSettings < Configuration

  %% Properties
  properties (SetAccess = {?ConfigSaveSettings, ?Configuration}, GetAccess = public)
    % Time interval for saving variables (should be larger than time-step)
    kVariableSavingTimeInterval_ (1, 1) double;
    % Save basic variables to csv file
    kSaveCsvFile_ (1, 1) logical = true;
    % Save the loaded config file in the dat folder if config is not "default"
    kSaveConfigFile_ (1, 1) logical = false;

    kSaveManipulability_ (1, 1) logical = false;
    kSaveDynamicManipulability_ (1, 1) logical = false;

    kSaveTumbleStabilityMargin_ (1, 1) logical = false;
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

      config_save_settings.kVariableSavingTimeInterval_ = config_world.time_step;

      if (config == "default")
        return;
      end

      config_save_settings = config_save_settings.override(config);

      % TODO: Check kVariableSavingTimeInterval_ is larger than time-step
    end

  end

  %% Getter
  methods (Access = public)

    function save_manipulability = getSaveManipulability(config_save_settings)
      save_manipulability = config_save_settings.kSaveManipulability_;
    end

    function save_dynamic_manipulability = getSaveDynamicManipulability(config_save_settings)
      save_dynamic_manipulability = config_save_settings.kSaveDynamicManipulability_;
    end

    function save_tumble_stability_margin = getSaveTumbleStabilityMargin(config_save_settings)
      save_tumble_stability_margin = config_save_settings.kSaveTumbleStabilityMargin_;
    end

  end

end  % ConfigSaveSettings
