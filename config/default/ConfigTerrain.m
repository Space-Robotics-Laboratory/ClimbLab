classdef ConfigTerrain  < Configuration
% Configuration for terrain parameters
%
% Created     : 2020.07.08 by Warley Ribeiro
% Last updated: 2025.01.06 by Masazumi Imai

  %% Properties for map
  properties (SetAccess = {?ConfigTerrain, ?Configuration}, GetAccess = public)
    surface_type (1, 1) string = "uneven";
      % ("flat_HR", "uneven")
    inclination (3, 1) double = [0.0; -0.0; 0.0];  % [deg]

    % Coefficients for calculating Ground Reaction Force
    stiffness_coefficient_for_GRF (1, 1) double = 10000.0;
    damping_coefficient_for_GRF (1, 1) double = 20.0;
    % Coefficients for calculating Ground Reaction Moment
    stiffness_coefficient_for_GRM (1, 1) double = 0.1;
    damping_coefficient_for_GRM (1, 1) double = 0.01;

    % Visualization
    surface_grid_color          = [0.9, 0.9, 0.9];  % RGB or color code
    surface_alpha (1, 1) double = 1.0;  % [0, 1]
  end

  %% Properties for Graspable Points
  properties (SetAccess = {?ConfigTerrain, ?Configuration}, GetAccess = public)
    % "all"
    graspable_points_detection_type (1, 1) string = "all";

    kVisualizeGraspablePoints_ (1, 1) logical = false;
      kGraspablePointsMarkerStyle_  (1, 1) string = "o";
      kGraspablePointsMarkerSize_   (1, 1) double = 10.0;
      kGraspablePointsColor_                      = [0.0, 0.0, 0.3];  % RGB or color code
      kGraspablePointsTransparency_ (1, 1) double = 0.1;  % [0, 1]
  end

  %% Constructor
  methods (Access = public)

    function config_terrain = ConfigTerrain(config)
    % ConfigTerrain() Constructor
    %   Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_terrain = config_terrain.override(config);

      % Convert color specifications to valid values
      config_terrain.surface_grid_color = validatecolor(config_terrain.surface_grid_color);
    end

  end

  %% Getter
  methods (Access = public)
    function surface_type = getSurfaceType(config_terrain)
      surface_type = config_terrain.surface_type;
    end
    function inclination = getSurfaceInclination(config_terrain)
      inclination = config_terrain.inclination;
    end
    function [GRF_stiffness, GRF_damping, GRM_stiffness, GRM_damping] = ...
        getGroundCoefficients(config_terrain)
      GRF_stiffness = config_terrain.stiffness_coefficient_for_GRF;
      GRF_damping = config_terrain.damping_coefficient_for_GRF;
      GRM_stiffness = config_terrain.stiffness_coefficient_for_GRM;
      GRM_damping = config_terrain.damping_coefficient_for_GRM;
    end
    function [surface_grid_color, surface_alpha] = getTerrainVisualSettings(config_terrain)
      surface_grid_color = config_terrain.surface_grid_color;
      surface_alpha = config_terrain.surface_alpha;
    end

    function graspable_points_detection_type = getGraspablePointsDetectionType(config_terrain)
      graspable_points_detection_type = config_terrain.graspable_points_detection_type;
    end
    function [visibility, marker_style, marker_size, color, transparency] = ...
        getGraspablePointsVisualSettings(config_terrain)
      visibility = config_terrain.kVisualizeGraspablePoints_;
      marker_style = config_terrain.kGraspablePointsMarkerStyle_;
      marker_size = config_terrain.kGraspablePointsMarkerSize_;
      color = config_terrain.kGraspablePointsColor_;
      transparency = config_terrain.kGraspablePointsTransparency_;
    end
  end

end  % ConfigTerrain
