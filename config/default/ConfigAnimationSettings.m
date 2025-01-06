classdef ConfigAnimationSettings < Configuration
% Configuration for animation settings
%
% Created:      2020.07.08 by Warley Ribeiro
% Last updated: 2024.12.25 by Masazumi Imai

  %% Properties
  properties (SetAccess = {?ConfigAnimationSettings, ?Configuration}, GetAccess = public)
    % General settings
    display_animation    (1, 1) logical = true;
    save_video           (1, 1) logical = false;
    video_file_extension (1, 1) string  = ".avi";      % (".avi", ".mp4")
    frame_rate           (1, 1) double  = 20;          % [frames/s] (positive value)
    resolution           (1, 2) double  = [640, 480];  % [px]
    show_elapsed_time    (1, 1) logical = true;

    font_name (1, 1) string = "Times New Roman";
    font_size (1, 1) double = 25;

    % Camera related
    x_axis_limit (1, 2) double = [-0.25, 0.25];  % [m]
    y_axis_limit (1, 2) double = [-0.25, 0.25];  % [m]
    z_axis_limit (1, 2) double = [-0.01, 0.25];  % [m]
    camera_azimuth (1, 1) double = -20;  % [deg]
    camera_elevation (1, 1) double = 12;  % [deg]
    camera_follow_robot (1, 1) logical = false;


    % Equilibrium related
    % Transformation from acceleration to visualize GIA Stable Region, GIA vector in the position coordinate
    % -> this scale is used for all accelerational variables visualization
    acceleration_expansion_factor (1, 1) double = 0.02;
    % Transformation from force to visualize Fg (gravitational force) and Fe (reaction force) Stable
    % Region in the position coordinate
    % -> this scale is used for all force-dimensional variables visualization
    force_expansion_factor = 0.04;

    gravitational_force_vec_show (1, 1) logical = false;
      gravitational_force_vec_color = [0.0, 0.7, 0.0];
      gravitational_force_vec_width (1, 1) double = 3.0;

    ground_reaction_force_vec_show (1, 1) logical = true;
      ground_reaction_force_vec_color = [1.0, 0.0, 0.0];
      ground_reaction_force_vec_width (1, 1) double = 3.0;

  end

  %% Constructor
  methods (Access = public)

    function config_animation_settings = ConfigAnimationSettings(config)
    % ConfigAnimationSettings() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_animation_settings = config_animation_settings.override(config);
    end

  end

end  % ConfigAnimationSettings
