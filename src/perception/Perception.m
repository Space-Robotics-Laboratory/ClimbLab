classdef Perception < handle
% Perception
%
% Created     : 2021.02.15 by Keigo Haji
% Last updated: 2025.01.07 by Masazumi Imai

%% Properties
  properties (SetAccess = immutable, GetAccess = private)
    kUseSensingCamera_ (1, 1) logical;
    kSensingType_ (1, 1) string;

    kMountingPosition_ (3, 1) double;  % [m]
    kMountingAngle_    (3, 1) double;  % (euler) [rad]

    kFOVHorizontal_  (1, 1) double;  % [rad]
    kFOVVertical_    (1, 1) double;  % [rad]
    kFOVMaxDistance_ (1, 1) double;  % [m]
    kFOVMinDistance_ (1, 1) double;  % [m]
  end
  properties (SetAccess = private, GetAccess = public)
    sensed_graspable_points_ (1, 1) GraspablePoints;
  end
  properties (SetAccess = private, GetAccess = private)
  end

  %% Public Methods
  methods (Access = public)

    function perception = Perception(config_perception, terrain, robot)
    % Constructor
      perception.kUseSensingCamera_ = config_perception.getUseSensingCamera();
      if (~perception.kUseSensingCamera_)
        return;
      end

      perception.kSensingType_ = config_perception.getSensingType();

      [perception.kMountingPosition_, perception.kMountingAngle_] = config_perception.getCameraMountingPose();
      [perception.kFOVHorizontal_, perception.kFOVVertical_, perception.kFOVMinDistance_, perception.kFOVMaxDistance_] = config_perception.getCameraFOVSettings();

      perception.sensed_graspable_points_ = GraspablePoints();
      [kVisibility, kMarkerStyle, kMarkerSize, kColor, kTransparency] = config_perception.getSensedGraspablePointsVisualSettings();
      perception.sensed_graspable_points_.setVisualSettings(kVisibility, kMarkerStyle, kMarkerSize, kColor, kTransparency);

      perception.initialSensing(config_perception, terrain, robot.getStateVariable());
    end

    function sensing(perception, terrain, robot)
      if (~perception.kUseSensingCamera_)
        return;
      end

      switch (perception.kSensingType_)
        case "RealSense_D435i"
          perception.sensingWithRealSenseD435i(terrain.getGraspablePoints(), robot.getStateVariable());
        otherwise
          error("ERROR: Failed sensing.")
      end
    end

    function visualize(perception)
      if (~perception.kUseSensingCamera_)
        return;
      end

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

    function sensingWithRealSenseD435i(perception, graspable_points, SV)
      % Calculation of Direction Cosine Matrix (Rotation matrix) of camera orientation in the World frame (coordinate system)
      %   W_R_C = W_R_B * B_R_C  (W: World, B: Base, C: Camera frame)
      base_orientation_DCM_in_World_frame = SV.getBaseOrientationDCM();  % W_R_B (Rotation matrix of Base in World frame)
      kCameraOrientationDCMInBaseFrame = rpy2dc(perception.kMountingAngle_);  % B_R_C
      camera_orientation_DCM_in_World_frame = base_orientation_DCM_in_World_frame * kCameraOrientationDCMInBaseFrame;  % W_R_C

      % Calculation of camera position in the World frame
      %   W_r_W->C = W_r_W->B + W_r_B->C = W_r_W->B + (W_R_B * B_r_B->C)
      base_position = SV.getBasePosition();  % W_r_W->B (Position vector (r) of Base from World in World frame)
      kCameraPositionInBaseFrame = perception.kMountingPosition_;  % B_r_B->C
      camera_position_in_World_frame = base_position + base_orientation_DCM_in_World_frame * kCameraPositionInBaseFrame;  % W_r_W->C

      % Calculation of graspable points position in Camera frame
      %   C_r_C->GP = C_R_W * W_r_C->GP = W_R_C^(-1) * (W_r_W->GP - W_r_W->C) = W_R_C^T * (W_r_W->GP - W_r_W->C)
      kGraspablePointsPositionInWorldFrame = graspable_points.getPointCloud();
      graspable_points_position_in_Camera_frame = camera_orientation_DCM_in_World_frame' * (kGraspablePointsPositionInWorldFrame - camera_position_in_World_frame);

      % Index of graspable points in the Camera frame within the distance limit of FOV in the x-axis
      GP_idx_in_FOV_x_distance_limit =  graspable_points_position_in_Camera_frame(1, :) > perception.kFOVMinDistance_ & ...
                                        graspable_points_position_in_Camera_frame(1, :) < perception.kFOVMaxDistance_;

      % Angle made in x-y coordinates
      theta_xy = atan(graspable_points_position_in_Camera_frame(2, :) ./ graspable_points_position_in_Camera_frame(1, :));
      % Index of graspable points in the Camera frame within the horizontal FOV of the camera
      GP_idx_in_horizontal_FOV =  theta_xy > -perception.kFOVHorizontal_ / 2.0 & ...
                                  theta_xy <  perception.kFOVHorizontal_ / 2.0;

      % Angle made in x-z coordinates
      theta_xz = atan(graspable_points_position_in_Camera_frame(3, :) ./ graspable_points_position_in_Camera_frame(1, :));
      % Index of graspable points in the Camera frame within the vertical FOV of the camera
      GP_idx_in_vertical_FOV =  theta_xz > -perception.kFOVVertical_ / 2.0 & ...
                                theta_xz <  perception.kFOVVertical_ / 2.0;

      % Index of graspable points in the Camera frame within the FOV of the camera
      GP_idx_in_FOV = GP_idx_in_FOV_x_distance_limit & GP_idx_in_horizontal_FOV & GP_idx_in_vertical_FOV;

      % Index of already sensed graspable points
      GP_idx_already = all(~isnan(perception.sensed_graspable_points_.getPointCloud()));

      % Index of sensed graspable points so far
      sensed_GP_idx = GP_idx_already | GP_idx_in_FOV;

      % Sensed graspable points
      sensed_graspable_points_position_in_World_frame = NaN(size(kGraspablePointsPositionInWorldFrame));
      sensed_graspable_points_position_in_World_frame(:, sensed_GP_idx) = kGraspablePointsPositionInWorldFrame(:, sensed_GP_idx);
      perception.sensed_graspable_points_.setSensedGraspablePoints(sensed_graspable_points_position_in_World_frame);
    end

  end

  %% Setter
  methods (Access = public)
  end

  %% Getter
  methods (Access = public)
  end

end  % Perception
