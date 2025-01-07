classdef ConfigPerception < Configuration
% Configuration for perception (sensing camera) parameters
%
% Created     : 2021.02.10 by Keigo Haji
% Last updated: 2025.01.07 by Masazumi Imai

  %% Properties
  properties (SetAccess = {?ConfigPerception, ?Configuration}, GetAccess = public)
    kUseSensingCamera_ (1, 1) logical = false;  % true/false

    % Sensing type
    % options: "RealSense_D435i"
    kSensingType_ (1, 1) string = "RealSense_D435i";

    % Mounting pose settings
    kMountingPosition_ (3, 1) double = [0.079; -0.011 ; 0.07];      % described in the Base frame [m] (Measured by CAD of Hubrobo v3.2)
    kMountingAngle_    (3, 1) double = deg2rad([0.0; -45.0; 0.0]);  % described in the Base frame (euler) [rad] (Measured by CAD of Hubrobo v3.2)

    % Field of View settings
    kFOVHorizontal_  (1, 1) double = deg2rad(86.0);  % [rad] (RealSense D435i)
    kFOVVertical_    (1, 1) double = deg2rad(57.0);  % [rad] (RealSense D435i)
    kFOVMinDistance_ (1, 1) double = 0.28;  % [m]   (RealSense D435i)
    kFOVMaxDistance_ (1, 1) double = 2.0;   % [m]   (RealSense D435i)

    % Initial known area settings
    % options: "circle"
    kInitialKnownAreaShape_ (1, 1) string = "circle";
      % Parameters for "circle"
      kCircularRadiusFromBaseCoM_ (1, 1) double = 0.4;  % [m]

    % Visualization settings for FOV of sensing camera
    kVisualizeSensingCameraFOV_ (1, 1) logical = false;
      % Marker for sensing camera position
      kSensingCameraMarkerStyle_ (1, 1) string = "o";
      kSensingCameraMarkerSize_  (1, 1) double = 1.0;
      kSensingCameraMarkerColor_               = "k";  % RGB or color code
      % Line for FOV area
      kSensingCameraFOVLineColor_               = "k";  % RGB or color code
      kSensingCameraFOVLineWidth_ (1, 1) double = 1.0;
      % Sensing camera FOV are surface settings
      kVisualizeSensingCameraFOVRegionSurface_   (1, 1) logical = false;
        kSensingCameraFOVFaceColor_                             = "c";  % RGB or color code
        kSensingCameraFOVFaceTransparency_       (1, 1) double  = 0.5;  % [0, 1]

    % Visualization settings for sensed graspable points
    kVisualizeSensedGraspablePoints_ (1, 1) logical = false;
      kSensedGraspablePointsMarkerStyle_  (1, 1) string = "o";
      kSensedGraspablePointsMarkerSize_   (1, 1) double = 10.0;
      kSensedGraspablePointsColor_                      = [1.0, 0.0, 1.0];  % RGB or color code
      kSensedGraspablePointsTransparency_ (1, 1) double = 1.0;  % [0, 1]
  end

  %% Constructor
  methods (Access = public)

    function config_perception = ConfigPerception(config)
    % Constructor
    % Override properties value based on specified config file if config is not "default"
      arguments (Input)
        config (1, 1) {mustBeA(config, "string")};
      end

      if (config == "default")
        return;
      end

      config_perception = config_perception.override(config);

      % Convert color specifications to valid values
      config_perception = config_perception.validateColor("kSensingCameraMarkerColor_");
      config_perception = config_perception.validateColor("kSensingCameraFOVLineColor_");
      config_perception = config_perception.validateColor("kSensingCameraFOVFaceColor_");
      config_perception = config_perception.validateColor("kSensedGraspablePointsColor_");
    end

  end

  %% Getter
  methods (Access = public)

    function kUseSensingCamera = getUseSensingCamera(config_perception)
      kUseSensingCamera = config_perception.kUseSensingCamera_;
    end

    function kSensingType = getSensingType(config_perception)
      kSensingType = config_perception.kSensingType_;
    end

    function [kMountingPosition, kMountingAngle] = getCameraMountingPose(config_perception)
      kMountingPosition = config_perception.kMountingPosition_;
      kMountingAngle    = config_perception.kMountingAngle_;
    end

    function [kFOVHorizontal, kFOVVertical, kFOVMinDistance, kFOVMaxDistance] = getCameraFOVSettings(config_perception)
      kFOVHorizontal  = config_perception.kFOVHorizontal_;
      kFOVVertical    = config_perception.kFOVVertical_;
      kFOVMinDistance = config_perception.kFOVMinDistance_;
      kFOVMaxDistance = config_perception.kFOVMaxDistance_;
    end

    function kInitialKnownAreaShape = getInitialKnownAreaShape(config_perception)
      kInitialKnownAreaShape = config_perception.kInitialKnownAreaShape_;
    end
    function kCircularRadiusFromBaseCoM = getCircularRadiusFromBaseCoM(config_perception)
      kCircularRadiusFromBaseCoM = config_perception.kCircularRadiusFromBaseCoM_;
    end

    function [kFOVVisibility, kMarkerStyle, kMarkerSize, kMarkerColor, ...
          kLineColor, kLineWidth, ...
          kFOVRegionSurfaceVisibility, kFaceColor, kFaceTransparency] = getSensingCameraFOVVisualSettings(config_perception)
      kFOVVisibility = config_perception.kVisualizeSensingCameraFOV_;
      kMarkerStyle = config_perception.kSensingCameraMarkerStyle_;
      kMarkerSize = config_perception.kSensedGraspablePointsMarkerSize_;
      kMarkerColor = config_perception.kSensingCameraMarkerColor_;
      kLineColor = config_perception.kSensingCameraFOVLineColor_;
      kLineWidth = config_perception.kSensingCameraFOVLineWidth_;
      kFOVRegionSurfaceVisibility = config_perception.kVisualizeSensingCameraFOVRegionSurface_;
      kFaceColor = config_perception.kSensingCameraFOVFaceColor_;
      kFaceTransparency = config_perception.kSensingCameraFOVFaceTransparency_;
    end

    function [visibility, marker_style, marker_size, color, transparency] = ...
          getSensedGraspablePointsVisualSettings(config_terrain)
      visibility   = config_terrain.kVisualizeSensedGraspablePoints_;
      marker_style = config_terrain.kSensedGraspablePointsMarkerStyle_;
      marker_size  = config_terrain.kSensedGraspablePointsMarkerSize_;
      color        = config_terrain.kSensedGraspablePointsColor_;
      transparency = config_terrain.kSensedGraspablePointsTransparency_;
    end

  end

end  % ConfigPerception
