classdef Configuration
% Configuration for terrain parameters
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2025.01.06 by Masazumi Imai

  %% Public Methods
  methods (Access = public)

    function default_config = override(default_config, user_config)
    % Override properties value based on specified config file if config is not "default"
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

      user_config_class = metaclass(kUserConfig);
      user_config_prop_list = user_config_class.PropertyList;

      for i = 1 : length(user_config_prop_list)
        kNumGetAccess = length(user_config_prop_list(i, 1).GetAccess);
        get_access_authorizations = strings(kNumGetAccess, 1);

        for j = 1 : kNumGetAccess
          get_access_authorizations(j, 1) = user_config_prop_list(i, 1).GetAccess{1, j}.Name;
        end

        if (any(get_access_authorizations == default_config_class_name))
          user_config_prop_name = user_config_prop_list(i, 1).Name;

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

    function default_config = validateColor(default_config, property_name, color)
    % Convert color specifications to valid values
      arguments (Input)
        default_config;
        property_name (1, 1) {mustBeA(property_name, "string")};
        color                {mustBeA(color, ["double", "string"])};
      end

      if (isstring(color) && color == "none")
        return;
      end
      default_config.(property_name) = validatecolor(color);
    end

  end

end  % Configuration
