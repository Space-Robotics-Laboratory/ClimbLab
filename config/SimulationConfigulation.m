classdef SimulationConfigulation < handle

  properties (SetAccess = private, GetAccess = public)
    environment_param ConfigEnvironmentParam
    robot_param ConfigRobotParam

    ani_settings ConfigAnimationSetting
  end

  methods (Access = public)
    % Constructor
    function config = SimulationConfigulation(config_type)
      arguments
        config_type string
      end
      config.environment_param = ConfigEnvironmentParam;
      config.robot_param = ConfigRobotParam;

      config.ani_settings = ConfigAnimationSetting;

      if config_type == "default"
        return;
      end
      config.override(config_type);

    end
  end

  methods (Access = private)
    function override(config, config_type)
      config_file_name = "config_" + config_type + "_param";
      if ~isfile("config\preset\" + config_file_name + ".m")
        error("Invalid config type is specified.");
      end
      config_file = str2func(config_file_name);
      feval(config_file);

      config.environment_param.override(environment_param);
      config.robot_param.override(robot_param);

      config.ani_settings.override(ani_settings);
    end
  end

end
% EOF