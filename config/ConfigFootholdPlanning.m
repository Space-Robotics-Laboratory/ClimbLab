classdef ConfigFootholdPlanning

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    % Foothold selection type
    % ("do_nothing", "fixed_stride")
    foothold_selection_type (1, 1) string = "fixed_stride";

    % For "fixed_stride"
    max_allowable_stride (1, 1) double = 0.05;  % [m]
    step_height (1, 1) double = 0.025;  % [m]
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

      kConfigFileName = "config_" + config;
      kPathToConfigFile = "config" + filesep + "preset" + filesep + kConfigFileName + ".m";
      if (~isfile(kPathToConfigFile))
        error("ERROR: The specified config file does NOT exist.");
      end

      kConfigFile = str2func(kConfigFileName);
      kUserConfig = feval(kConfigFile);

      kDefaultConfigPropName = properties(config_foothold_planning);

      meta_class = metaclass(kUserConfig);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigFootholdPlanning"))
          kUserConfigPropName = meta_props(i, 1).Name;

          if (~any(strcmp(kDefaultConfigPropName, kUserConfigPropName)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + kUserConfigPropName + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          config_foothold_planning.(kUserConfigPropName) = kUserConfig.(kUserConfigPropName);
        end
      end

      % TODO: This should be delete
      if (config_foothold_planning.foothold_selection_type ~= "fixed_stride")
        config_foothold_planning.max_allowable_stride = NaN;
      end
    end

  end

  %% Getter
  methods (Access = public)
    function foothold_selection_type = getFootholdSelectionType(config_foothold_planning)
      foothold_selection_type = config_foothold_planning.foothold_selection_type;
    end
    function max_allowable_stride = getMaxAllowableStride(config_foothold_planning)
      max_allowable_stride = config_foothold_planning.max_allowable_stride;
    end
    function step_height = getStepHeight(config_foothold_planning)
      step_height = config_foothold_planning.step_height;
    end
  end
end  % ConfigFootholdPlanning
