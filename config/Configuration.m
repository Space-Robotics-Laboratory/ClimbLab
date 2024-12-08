classdef Configuration

  %% Public Methods
  methods (Access = public)

    function default_config = override(default_config, user_config)
    % override()
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        default_config;
        user_config (1, 1) {mustBeA(user_config, "string")};
      end


      kConfigFileName = "config_" + user_config;
      kPathToConfigFile = "config" + filesep + "preset" + filesep + kConfigFileName + ".m";
      if (~isfile(kPathToConfigFile))
        error("ERROR: The specified config file does NOT exist.");
      end

      kConfigFile = str2func(kConfigFileName);
      kUserConfig = feval(kConfigFile);

      default_config_class_name = class(default_config);
      default_config_prop_name = properties(default_config);

      meta_class = metaclass(kUserConfig);
      meta_props = meta_class.PropertyList;

      for i = 1 : length(meta_props)
        get_access_authorization = meta_props(i, 1).GetAccess{1, 2}.Name;

        if (strcmp(get_access_authorization, default_config_class_name))
          user_config_prop_name = meta_props(i, 1).Name;

          if (~any(strcmp(default_config_prop_name, user_config_prop_name)))
            error("ERROR: Invalid property name is specified in user customized config file. " + ...
              "That property name is """ + user_config_prop_name + """. " + ...
              "Property name defined in user customized config file have to match " + ...
              "default config property name.");
          end

          default_config.(user_config_prop_name) = kUserConfig.(user_config_prop_name);
        end
      end

    end

  end

end  % Configuration
