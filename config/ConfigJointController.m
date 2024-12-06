classdef ConfigJointController
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    type (1, 1) string = "PD_control";

    proportional_gain (1, 1) double = 30.0;
    derivative_gain   (1, 1) double = 0.2;
  end

  %% Constructor
  methods (Access = public)

    function config_joint_controller = ConfigJointController(config)
    % ConfigJointController() Constructor
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

      this_config_prop_name = properties(config_joint_controller);

      meta_class = metaclass(user_config);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 1}.Name;

        if (strcmp(get_access_authorization, "ConfigJointController"))
          user_config_prop_name = meta_props(i, 1).Name;

          if (~any(strcmp(this_config_prop_name, user_config_prop_name)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + user_config_prop_name + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          config_joint_controller.(user_config_prop_name) = user_config.(user_config_prop_name);
        end
      end
    end

  end

  %% Getter
  methods (Access = public)
    function type = getType(config_control_param)
      type = config_control_param.type;
    end
    function [proportional_gain, derivative_gain] = getPDControllerGain(config_control_param)
      proportional_gain = config_control_param.proportional_gain;
      derivative_gain = config_control_param.derivative_gain;
    end
  end

end
% EOF