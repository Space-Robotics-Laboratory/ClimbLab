classdef Perception < handle
% Perception
%
% Created     : 2021.02.15 by Keigo Haji
% Last updated: 2025.01.06 by Masazumi Imai

%% Properties
  properties (SetAccess = private, GetAccess = public)
    sensed_graspable_points_ (1, 1) GraspablePoints;
  end
  properties (SetAccess = private, GetAccess = private)
  end

  %% Public Methods
  methods (Access = public)

    function perception = Perception(config_perception, terrain, robot)
    % Constructor
      perception.sensed_graspable_points_ = GraspablePoints();
      [kVisibility, kMarkerStyle, kMarkerSize, kColor, kTransparency] = config_perception.getSensedGraspablePointsVisualSettings();
      perception.sensed_graspable_points_.setVisualSettings(kVisibility, kMarkerStyle, kMarkerSize, kColor, kTransparency);

      perception.initialSensing(config_perception, terrain, robot.getStateVariable());
    end

    function visualize(perception)
      perception.sensed_graspable_points_.visualize();
    end

  end

  %% Private Methods
  methods (Access = private)

    function initialSensing(perception, config_perception, terrain, SV)
      kAllGraspablePointsPositionInWorldFrame = terrain.getGraspablePoints().getPointCloud();
      kInitialBasePositionInWorldFrame = SV.getBasePosition();
      kProjectionPointOfInitialBasePositionInWorldFrame = terrain.getProjectionPointInWorldFrame(kInitialBasePositionInWorldFrame);
      sensed_graspable_points_position = NaN(size(kAllGraspablePointsPositionInWorldFrame));

      switch (config_perception.getInitialKnownAreaShape())
        case "circle"
          kCircularRadiusFromBaseCoM = config_perception.getCircularRadiusFromBaseCoM();
          graspable_points_position_from_base_CoM = kAllGraspablePointsPositionInWorldFrame - kProjectionPointOfInitialBasePositionInWorldFrame;
          distance_from_base_CoM = vecnorm(graspable_points_position_from_base_CoM, 2, 1);
          idx_in_known_area = distance_from_base_CoM < kCircularRadiusFromBaseCoM;
          sensed_graspable_points_position(:, idx_in_known_area) = kAllGraspablePointsPositionInWorldFrame(:, idx_in_known_area);

        otherwise
          error("ERROR: Failed to initialize sensed graspable points.");
      end

      perception.sensed_graspable_points_.setSensedGraspablePoints(sensed_graspable_points_position);
    end

  end

  %% Setter
  methods (Access = public)
  end

  %% Getter
  methods (Access = public)
  end

end  % Perception
