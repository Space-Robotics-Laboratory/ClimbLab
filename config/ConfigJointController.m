classdef ConfigJointController < Configuration
  %% Properties
  properties (SetAccess = {?ConfigJointController, ?Configuration}, GetAccess = public)
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

      config_joint_controller = config_joint_controller.override(config);
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

end  % ConfigJointController
