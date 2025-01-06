classdef ConfigSaveSettings < Configuration
% Configuration for variables save settings
%
% Created     : 2020.07.08 by Warley Ribeiro
% Last updated: 2025.01.05 by Masazumi Imai

  %% Properties
  properties (SetAccess = {?ConfigSaveSettings, ?Configuration}, GetAccess = public)
    % Save basic variables to csv file
    kSaveCsvFile_ (1, 1) logical = true;
    % Save the loaded config file in the dat folder if config is not "default"
    kSaveConfigFile_ (1, 1) logical = false;

    % Time interval for saving variables (should be larger than time-step)
    kVariableSavingTimeInterval_ (1, 1) double;

    % Maximum of absolute torque of all joint
    kSaveMaxJointTorque_ (1, 1) logical = false;
    % Root Mean Square (RMS) of torque of all joint
    % NOTE: Need "Signal Processing Toolbox" if MATLAB version is before R2022a
    kSaveRMSJointTorque_ (1, 1) logical = false;

    kSaveManipulability_ (1, 1) logical = false;
    kSaveDynamicManipulability_ (1, 1) logical = false;

    kSaveTumbleStabilityMargin_ (1, 1) logical = false;

    kSaveGravitoInertialAcceleration_ (1, 1) logical = false;

    kSaveCostOfTransport_ (1, 1) logical = false;
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

      config_save_settings.kVariableSavingTimeInterval_ = config_world.getTimeStep();

      if (config == "default")
        return;
      end

      config_save_settings = config_save_settings.override(config);

      % TODO: Check kVariableSavingTimeInterval_ is larger than time-step
    end

  end

end  % ConfigSaveSettings
