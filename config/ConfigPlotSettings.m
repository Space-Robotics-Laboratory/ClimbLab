classdef ConfigPlotSettings < Configuration

  %% Properties
  properties (SetAccess = {?ConfigPlotSettings, ?Configuration}, GetAccess = public)
    kFontName_ (1, 1) string = "Times New Roman";
    kFontSize_ (1, 1) double = 25.0;

    kLineWidth_ (1, 1) double = 3.0;

    kSaveGraphs_ (1, 1) logical = false;

    kPlotBasePosition_ (1, 1) logical = true;

    kPlotJointTorque_ (1, 1) logical = true;

    kPlotTumbleStabilityMargin_ (1, 1) logical = true;
  end

  %% Public Methods
  methods (Access = public)

    function config_plot_settings = ConfigPlotSettings(config)
    % ConfigSaveSettings() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_plot_settings = config_plot_settings.override(config);
    end

  end

end  % ConfigSaveSettings
