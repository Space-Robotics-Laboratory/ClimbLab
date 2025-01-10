classdef ConfigFootholdPlanning < Configuration
% Configuration for foothold planning parameters
%
% Created     : 2020.07.08 by Warley Ribeiro
% Last updated: 2025.01.09 by Masazumi Imai

  %% Properties
  properties (SetAccess = {?ConfigFootholdPlanning, ?Configuration}, GetAccess = public)
    % Foothold selection type
    % ("do_nothing", "fixed_stride")
    foothold_selection_type (1, 1) string = "fixed_stride";

    % For "fixed_stride"
    kAllowableMaxStride_ (1, 1) double = 0.05;  % [m]
  end

  %% Constructor
  methods (Access = public)

    function config_foothold_planning = ConfigFootholdPlanning(config)
    % ConfigFootholdPlanning() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end


      config_foothold_planning = config_foothold_planning.override(config);
    end

  end

  %% Getter
  methods (Access = public)
    function foothold_selection_type = getFootholdSelectionType(config_foothold_planning)
      foothold_selection_type = config_foothold_planning.foothold_selection_type;
    end
    function kAllowableMaxStride_ = getAllowableMaxStride(config_foothold_planning)
      kAllowableMaxStride_ = config_foothold_planning.kAllowableMaxStride_;
    end
  end
end  % ConfigFootholdPlanning
