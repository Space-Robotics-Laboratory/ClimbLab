classdef Perception < handle
% Perception
%
% Created     : 2021.02.15 by Keigo Haji
% Last updated: 2025.01.09 by Masazumi Imai

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
    kVisualizeSensingCameraFOV_ (1, 1) logical;
    kVisualizeSensingCameraFOVRegionSurface_ (1, 1) logical;

    graphics_camera_mounting_position_               (1, 1) matlab.graphics.chart.primitive.Scatter;
    graphics_rectangles_for_scan_range_              (2, 1) matlab.graphics.chart.primitive.Line;
    graphics_lines_connecting_correspond_end_points_ (4, 1) matlab.graphics.chart.primitive.Line;
    graphics_FOV_region_surface_                     (6, 1) matlab.graphics.primitive.Patch;
  end

  %% Public Methods
  methods (Access = public)

    function perception = Perception(config_perception, terrain, robot)
    % Constructor
      arguments (Input)
        config_perception (1, 1) {mustBeA(config_perception, "ConfigPerception")};
        terrain           (1, 1) {mustBeA(terrain,           "Terrain")};
        robot             (1, 1) {mustBeA(robot,             "Robot")};
      end

      perception.kUseSensingCamera_ = config_perception.getUseSensingCamera();
      if (~perception.kUseSensingCamera_)
        return;
      end

      perception.kSensingType_ = config_perception.getSensingType();

      % Sensing camera FOV
      [perception.kMountingPosition_, perception.kMountingAngle_] = config_perception.getCameraMountingPose();
      [perception.kFOVHorizontal_, perception.kFOVVertical_, perception.kFOVMinDistance_, perception.kFOVMaxDistance_] = config_perception.getCameraFOVSettings();
      [perception.kVisualizeSensingCameraFOV_, kMarkerStyle, kMarkerSize, kMarkerColor, ...
        kLineColor, kLineWidth, ...
        perception.kVisualizeSensingCameraFOVRegionSurface_, kFaceColor, kFaceTransparency] = config_perception.getSensingCameraFOVVisualSettings();
      switch (perception.kSensingType_)
        case "RealSense_D435i"
          perception.createSensingCameraFOVGraphics(kMarkerStyle, kMarkerSize, kMarkerColor, ...
            kLineColor, kLineWidth, kFaceColor, kFaceTransparency);
        otherwise
          error("ERROR: Failed to create graphics of sensing camera FOV.")
      end

      % Sensed graspable points
      perception.sensed_graspable_points_ = GraspablePoints();
      [kVisibility, kMarkerStyle, kMarkerSize, kColor, kTransparency] = config_perception.getSensedGraspablePointsVisualSettings();
      perception.sensed_graspable_points_.setVisualSettings(kVisibility, kMarkerStyle, kMarkerSize, kColor, kTransparency);

      perception.initialSensing(config_perception, terrain, robot.getStateVariable());
    end

    function sensing(perception, terrain, robot)
      arguments (Input)
        perception;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
        robot   (1, 1) {mustBeA(robot,   "Robot")};
      end

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

    function visualize(perception, robot)
      arguments (Input)
        perception;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      if (~perception.kUseSensingCamera_)
        return;
      end

      if (perception.kVisualizeSensingCameraFOV_)
        switch (perception.kSensingType_)
          case "RealSense_D435i"
            perception.visualizeSensingCameraFOVWithRealSenseD435i(robot.getStateVariable());
        otherwise
          error("ERROR: Failed to visualize FOV of sensing camera.")
        end
      end

      perception.sensed_graspable_points_.visualize();
    end

  end

  %% Private Methods
  methods (Access = private)

    function initialSensing(perception, config_perception, terrain, SV)
      arguments (Input)
        perception;
        config_perception (1, 1) {mustBeA(config_perception, "ConfigPerception")};
        terrain           (1, 1) {mustBeA(terrain,           "Terrain")};
        SV                (1, 1) {mustBeA(SV,                "StateVariable")};
      end

      kAllGraspablePointsPositionInWorldFrame = terrain.getGraspablePoints().getPointCloud();
      kInitialBasePositionInWorldFrame = SV.getBasePosition();
      kProjectionPointOfInitialBasePositionInWorldFrame = terrain.getProjectionPointInWorldFrameInZDirOfGroundFrame(kInitialBasePositionInWorldFrame);
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
      arguments (Input)
        perception;
        graspable_points (1, 1) {mustBeA(graspable_points, "GraspablePoints")};
        SV               (1, 1) {mustBeA(SV,               "StateVariable")};
      end

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

    function createSensingCameraFOVGraphics(perception, kMarkerType, kMarkerSize, kMarkerColor, ...
        kLineColor, kLineWidth, kFaceColor, kFaceTransparency)
      arguments (Input)
        perception;
        kMarkerType       (1, 1) {mustBeA(kMarkerType,       "string")};
        kMarkerSize       (1, 1) {mustBeA(kMarkerSize,       "double")};
        kMarkerColor             {mustBeA(kMarkerColor,      ["double", "string"])};
        kLineColor               {mustBeA(kLineColor,        ["double", "string"])};
        kLineWidth        (1, 1) {mustBeA(kLineWidth,        "double")};
        kFaceColor               {mustBeA(kFaceColor,        ["double", "string"])};
        kFaceTransparency (1, 1) {mustBeA(kFaceTransparency, "double")};
      end

      if (~perception.kVisualizeSensingCameraFOV_)
        return;
      end

      % Camera mounting position
      camera_position_in_World_frame = [0.0; 0.0; 0.0];  % for initialization
      perception.graphics_camera_mounting_position_ = scatter3(...
        camera_position_in_World_frame(1, 1), camera_position_in_World_frame(2, 1), camera_position_in_World_frame(3, 1), ...
        kMarkerSize, kMarkerColor, kMarkerType, "filled", Visible = "off");

      % Two rectangles connecting end points (also end points forming FOV)
      end_points_for_min_range_in_World_frame = zeros(3, 4);
      end_points_for_max_range_in_World_frame = zeros(3, 4);
      extended_matrix_of_end_points_for_min_range_for_plotting = zeros(3, 4 + 1);
      extended_matrix_of_end_points_for_max_range_for_plotting = zeros(3, 4 + 1);
      rectangles_for_scan_range_tmp(1, 1) = plot3(...
        extended_matrix_of_end_points_for_min_range_for_plotting(1, :), extended_matrix_of_end_points_for_min_range_for_plotting(2, :), extended_matrix_of_end_points_for_min_range_for_plotting(3, :), ...
        Color = kLineColor, LineWidth = kLineWidth, Marker = kMarkerType, MarkerSize = kMarkerSize, MarkerFaceColor = kMarkerColor, Visible = "off");
      rectangles_for_scan_range_tmp(2, 1) = plot3(...
        extended_matrix_of_end_points_for_max_range_for_plotting(1, :), extended_matrix_of_end_points_for_max_range_for_plotting(2, :), extended_matrix_of_end_points_for_max_range_for_plotting(3, :), ...
        Color = kLineColor, LineWidth = kLineWidth, Marker = kMarkerType, MarkerSize = kMarkerSize, MarkerFaceColor = kMarkerColor, Visible = "off");
      perception.graphics_rectangles_for_scan_range_ = rectangles_for_scan_range_tmp;

      % Four lines connecting correspond end points between the near and far side
      kNumEndPointsOnOneSide = size(end_points_for_min_range_in_World_frame, 2);
      corresponded_end_points = zeros(3, 2, kNumEndPointsOnOneSide);
      lines_connecting_correspond_end_points_tmp = matlab.graphics.chart.primitive.Line.empty;
      for i = 1 : kNumEndPointsOnOneSide
        corresponded_end_points(:, :, i) = [end_points_for_min_range_in_World_frame(:, i), end_points_for_max_range_in_World_frame(:, i)];
        lines_connecting_correspond_end_points_tmp(i, 1) = plot3(...
          corresponded_end_points(1, :, i), corresponded_end_points(2, :, i), corresponded_end_points(3, :, i), ...
          Color = kLineColor, LineWidth = kLineWidth, Marker = kMarkerType, MarkerSize = kMarkerSize, MarkerFaceColor = kMarkerColor, Visible = "off");
      end
      perception.graphics_lines_connecting_correspond_end_points_ = lines_connecting_correspond_end_points_tmp;

      % FOV region surface
      if (~perception.kVisualizeSensingCameraFOVRegionSurface_)
        return;
      end
      end_points_min = end_points_for_min_range_in_World_frame;
      end_points_max = end_points_for_max_range_in_World_frame;
      FOV_region_surface_tmp(1, 1) = patch(end_points_min(1, :), end_points_min(2, :), end_points_min(3, :), FaceColor = kFaceColor, EdgeColor = "none", FaceAlpha = kFaceTransparency, Visible = "off");
      FOV_region_surface_tmp(2, 1) = patch(end_points_max(1, :), end_points_max(2, :), end_points_max(3, :), FaceColor = kFaceColor, EdgeColor = "none", FaceAlpha = kFaceTransparency, Visible = "off");
      for i = 1 : kNumEndPointsOnOneSide
        if (i == kNumEndPointsOnOneSide)
          j = 1;
        else
          j = i + 1;
        end
        end_points_x = [end_points_min(1, i), end_points_min(1, j), end_points_max(1, j), end_points_max(1, i)];
        end_points_y = [end_points_min(2, i), end_points_min(2, j), end_points_max(2, j), end_points_max(2, i)];
        end_points_z = [end_points_min(3, i), end_points_min(3, j), end_points_max(3, j), end_points_max(3, i)];
        FOV_region_surface_tmp(2 + i, 1) = patch(end_points_x, end_points_y, end_points_z, FaceColor = kFaceColor, EdgeColor = "none", FaceAlpha = kFaceTransparency, Visible = "off");
      end
      perception.graphics_FOV_region_surface_ = FOV_region_surface_tmp;
    end

    function visualizeSensingCameraFOVWithRealSenseD435i(perception, SV)
      arguments (Input)
        perception;
        SV (1, 1) {mustBeA(SV, "StateVariable")};
      end

      base_position = SV.getBasePosition();
      base_orientation_DCM_in_World_frame = SV.getBaseOrientationDCM();
      kCameraPositionInBaseFrame = perception.kMountingPosition_;

      camera_position_in_World_frame = base_position + base_orientation_DCM_in_World_frame * kCameraPositionInBaseFrame;

      kFOVHorizontal = perception.kFOVHorizontal_;
      kFOVVertical = perception.kFOVVertical_;
      % Camera range on the near side
      kFOVMinDistance = perception.kFOVMinDistance_;
      scan_range_near_y_minus = kFOVMinDistance * tan(-kFOVHorizontal / 2.0);  % Boundary in direction of minus y-axis of Camera frame
      scan_range_near_y_plus  = kFOVMinDistance * tan( kFOVHorizontal / 2.0);  % Boundary in direction of plus y-axis of Camera frame
      scan_range_near_z_minus = kFOVMinDistance * tan(-kFOVVertical   / 2.0);
      scan_range_near_z_plus  = kFOVMinDistance * tan( kFOVVertical   / 2.0);
      % Camera range on the far side
      kFOVMaxDistance = perception.kFOVMaxDistance_;
      scan_range_far_y_minus = kFOVMaxDistance * tan(-kFOVHorizontal / 2.0);
      scan_range_far_y_plus  = kFOVMaxDistance * tan( kFOVHorizontal / 2.0);
      scan_range_far_z_minus = kFOVMaxDistance * tan(-kFOVVertical   / 2.0);
      scan_range_far_z_plus  = kFOVMaxDistance * tan( kFOVVertical   / 2.0);

      % End points position that form the FOV in the Camera frame.
      % The four points on the near side and the far side are set separately.
      %   1st dim: x-y-z coordinate
      %   2nd dim: End points
      end_points_for_min_range_in_Camera_frame = [kFOVMinDistance,         kFOVMinDistance,        kFOVMinDistance,         kFOVMinDistance;
                                                  scan_range_near_y_minus, scan_range_near_y_plus, scan_range_near_y_plus,  scan_range_near_y_minus;
                                                  scan_range_near_z_plus,  scan_range_near_z_plus, scan_range_near_z_minus, scan_range_near_z_minus];
      end_points_for_max_range_in_Camera_frame = [kFOVMaxDistance,          kFOVMaxDistance,         kFOVMaxDistance,          kFOVMaxDistance;
                                                  scan_range_far_y_minus, scan_range_far_y_plus, scan_range_far_y_plus,  scan_range_far_y_minus;
                                                  scan_range_far_z_plus,  scan_range_far_z_plus, scan_range_far_z_minus, scan_range_far_z_minus];
      % Transform end points from the Camera frame to the World frame
      kCameraOrientationDCMInBaseFrame = rpy2dc(perception.kMountingAngle_);
      end_points_for_min_range_in_World_frame = camera_position_in_World_frame + base_orientation_DCM_in_World_frame * kCameraOrientationDCMInBaseFrame * end_points_for_min_range_in_Camera_frame;
      end_points_for_max_range_in_World_frame = camera_position_in_World_frame + base_orientation_DCM_in_World_frame * kCameraOrientationDCMInBaseFrame * end_points_for_max_range_in_Camera_frame;

      % Update camera mounting position
      perception.graphics_camera_mounting_position_.XData = camera_position_in_World_frame(1, 1);
      perception.graphics_camera_mounting_position_.YData = camera_position_in_World_frame(2, 1);
      perception.graphics_camera_mounting_position_.ZData = camera_position_in_World_frame(3, 1);
      perception.graphics_camera_mounting_position_.Visible = "on";

      % Update two rectangles connecting the end points
      extended_matrix_of_end_points_for_min_range_for_plotting = [end_points_for_min_range_in_World_frame, end_points_for_min_range_in_World_frame(:, 1)];
      perception.graphics_rectangles_for_scan_range_(1, 1).XData = extended_matrix_of_end_points_for_min_range_for_plotting(1, :);
      perception.graphics_rectangles_for_scan_range_(1, 1).YData = extended_matrix_of_end_points_for_min_range_for_plotting(2, :);
      perception.graphics_rectangles_for_scan_range_(1, 1).ZData = extended_matrix_of_end_points_for_min_range_for_plotting(3, :);
      perception.graphics_rectangles_for_scan_range_(1, 1).Visible = "on";
      extended_matrix_of_end_points_for_max_range_for_plotting = [end_points_for_max_range_in_World_frame, end_points_for_max_range_in_World_frame(:, 1)];
      perception.graphics_rectangles_for_scan_range_(2, 1).XData = extended_matrix_of_end_points_for_max_range_for_plotting(1, :);
      perception.graphics_rectangles_for_scan_range_(2, 1).YData = extended_matrix_of_end_points_for_max_range_for_plotting(2, :);
      perception.graphics_rectangles_for_scan_range_(2, 1).ZData = extended_matrix_of_end_points_for_max_range_for_plotting(3, :);
      perception.graphics_rectangles_for_scan_range_(2, 1).Visible = "on";

      % Update four lines connecting correspond end points between the near and far side
      kNumEndPointsOnOneSide = size(end_points_for_min_range_in_World_frame, 2);
      corresponded_end_points = zeros(3, 2, kNumEndPointsOnOneSide);
      for i = 1 : kNumEndPointsOnOneSide
        corresponded_end_points(:, :, i) = [end_points_for_min_range_in_World_frame(:, i), end_points_for_max_range_in_World_frame(:, i)];
        perception.graphics_lines_connecting_correspond_end_points_(i, 1).XData = corresponded_end_points(1, :, i);
        perception.graphics_lines_connecting_correspond_end_points_(i, 1).YData = corresponded_end_points(2, :, i);
        perception.graphics_lines_connecting_correspond_end_points_(i, 1).ZData = corresponded_end_points(3, :, i);
        perception.graphics_lines_connecting_correspond_end_points_(i, 1).Visible = "on";
      end

      % Update FOV region surface
      if (~perception.kVisualizeSensingCameraFOVRegionSurface_)
        return;
      end
      end_points_min = end_points_for_min_range_in_World_frame;
      end_points_max = end_points_for_max_range_in_World_frame;
      perception.graphics_FOV_region_surface_(1, 1).XData = end_points_min(1, :);
      perception.graphics_FOV_region_surface_(1, 1).YData = end_points_min(2, :);
      perception.graphics_FOV_region_surface_(1, 1).ZData = end_points_min(3, :);
      perception.graphics_FOV_region_surface_(1, 1).Visible = "on";
      perception.graphics_FOV_region_surface_(2, 1).XData = end_points_max(1, :);
      perception.graphics_FOV_region_surface_(2, 1).YData = end_points_max(2, :);
      perception.graphics_FOV_region_surface_(2, 1).ZData = end_points_max(3, :);
      perception.graphics_FOV_region_surface_(2, 1).Visible = "on";
      for i = 1 : kNumEndPointsOnOneSide
        if (i == kNumEndPointsOnOneSide)
          j = 1;
        else
          j = i + 1;
        end
        end_points_x = [end_points_min(1, i), end_points_min(1, j), end_points_max(1, j), end_points_max(1, i)];
        end_points_y = [end_points_min(2, i), end_points_min(2, j), end_points_max(2, j), end_points_max(2, i)];
        end_points_z = [end_points_min(3, i), end_points_min(3, j), end_points_max(3, j), end_points_max(3, i)];
        perception.graphics_FOV_region_surface_(2 + i, 1).XData = end_points_x;
        perception.graphics_FOV_region_surface_(2 + i, 1).YData = end_points_y;
        perception.graphics_FOV_region_surface_(2 + i, 1).ZData = end_points_z;
        perception.graphics_FOV_region_surface_(2 + i, 1).Visible = "on";
      end
    end

  end

  %% Getter
  % methods (Access = public)

  %   function kUseSensingCamera = getUseSensingCamera(perception)
  %     kUseSensingCamera = perception.kUseSensingCamera_;
  %   end

  %   function sensed_graspable_points = getSensedGraspablePoints(perception)
  %     sensed_graspable_points = perception.sensed_graspable_points_;
  %   end

  % end

end  % Perception
