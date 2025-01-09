classdef GraspablePoints < handle
% Graspable points parameters
%
% Created     : 2020.05.12 by Yusuke Koizumi
% Last updated: 2025.01.09 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kDetectionType_ (1, 1) string;
    point_cloud_    (3, :) double;  % Graspable points position described in world frame [m]
  end
  properties (SetAccess = private, GetAccess = private)
    graphics_ (1, 1) matlab.graphics.chart.primitive.Scatter;
    kVisualization_ (1, 1) logical;
    kMarkerStyle_ (1, 1) string;
    kMarkerSize_ (1, 1) double;
    kColor_ (1, 3) double;  % [0, 1]
    kTransparency_ (1, 1) double;  % [0, 1]
  end

  %% Public Methods
  methods (Access = public)

    function graspable_points = GraspablePoints()
    % Constructor
      graspable_points.kDetectionType_ = strings;
    end

    function setVisualSettings(graspable_points, visibility, marker_style, marker_size, color, transparency)
      arguments (Input)
        graspable_points;
        visibility   (1, 1) {mustBeA(visibility, "logical")};
        marker_style (1, 1) {mustBeA(marker_style, "string")};
        marker_size  (1, 1) {mustBeA(marker_size, "double")};
        color        (:, 1) {mustBeA(color, ["double", "string"])};
        transparency (1, 1) {mustBeA(transparency, "double")};
      end

      graspable_points.kVisualization_ = visibility;
      graspable_points.kMarkerStyle_ = marker_style;
      graspable_points.kMarkerSize_ = marker_size;
      graspable_points.kColor_ = color;
      graspable_points.kTransparency_ = transparency;

      if (graspable_points.kVisualization_)
        graspable_points.createGraspablePointsGraphics();
      end
    end

    function visualize(graspable_points)
      if (graspable_points.kVisualization_)
        graspable_points.graphics_.XData = graspable_points.point_cloud_(1, :);
        graspable_points.graphics_.YData = graspable_points.point_cloud_(2, :);
        graspable_points.graphics_.ZData = graspable_points.point_cloud_(3, :);
        graspable_points.graphics_.Visible = "on";
      end
    end

  end

  %% Methods called only from Terrain
  methods (Access = ?Terrain)

    function setDetectionType(graspable_points, config_terrain)
      arguments (Input)
        graspable_points;
        config_terrain (1, 1) {mustBeA(config_terrain, "ConfigTerrain")};
      end

      graspable_points.kDetectionType_ = config_terrain.getGraspablePointsDetectionType();
    end

    function setGraspablePoints(graspable_points, terrain_points)
      arguments (Input)
        graspable_points;
        terrain_points (3, :) {mustBeA(terrain_points, "double")};
      end

      switch graspable_points.kDetectionType_
        case "all"
          point_cloud = terrain_points;
        otherwise
          error("ERROR: Failed to set graspable points.");
      end

      graspable_points.point_cloud_ = point_cloud;
    end

  end

  %% Methods called only from Perception
  methods (Access = ?Perception)

    function setSensedGraspablePoints(graspable_points, sensed_graspable_points)
      arguments (Input)
        graspable_points;
        sensed_graspable_points (3, :) {mustBeA(sensed_graspable_points, "double")};
      end
      graspable_points.point_cloud_ = sensed_graspable_points;
    end

  end

  %% Methods called only from FootholdPlanning
  methods (Access = ?FootholdPlanning)

    function graspable_points_in_reachable_area = updateGraspablePointsInReachableArea(graspable_points, terrain, robot, perception)
      arguments (Input)
        graspable_points;
        terrain       (1, 1) {mustBeA(terrain,    "Terrain")};
        robot         (1, 1) {mustBeA(robot,      "Robot")};
        perception    (1, 1) {mustBeA(perception, "Perception")};
      end

      LP = robot.getLinkParameter();
      SV = robot.getStateVariable();
      reachable_area = robot.getKinematics().getReachableArea();

      [kMinJointLimit, kMaxJointLimit] = LP.getJointLimit();
      kMinJointLimitRad = deg2rad(kMinJointLimit);
      kMaxJointLimitRad = deg2rad(kMaxJointLimit);
      kJoints = LP.getJoints();

      base_position_in_World_frame = SV.getBasePosition();
      base_orientation_DCM_in_World_frame = SV.getBaseOrientationDCM();
      proj_point_base_in_World_frame = terrain.getProjectionPointInWorldFrameInZDirOfGroundFrame(base_position_in_World_frame);

      % Graspable points position in the World frame
      if (perception.getUseSensingCamera())
        graspable_points_in_World_frame = perception.getSensedGraspablePoints().getPointCloud();
      else
        graspable_points_in_World_frame = terrain.getGraspablePoints().getPointCloud();
      end

      % Calculation of position of the first joint for each limb in the World frame
      c0 = LP.getPositionVectorFromBaseCoMToJoint();
      kNumLimb = LP.getNumberOfLimb();
      kNumJointsPerLimb = LP.getNumberOfJointsPerLimb();
      first_joint_position_in_World_frame = base_position_in_World_frame + base_orientation_DCM_in_World_frame * c0(:, kNumJointsPerLimb(1, :) .* ((1 : kNumLimb) - 1) + 1);

      % Projected point of position of the first joint for each limb in the World frame
      proj_point_j1_in_the_World_frame = terrain.getProjectionPointInWorldFrameInZDirOfGroundFrame(first_joint_position_in_World_frame);
      % Vector from first joint position to graspable points position projected on the terrain surface in the World frame
      proj_vec_j1_to_GP = NaN(size(graspable_points_in_World_frame));
      for limb_id = 1 : kNumLimb
        proj_vec_j1_to_GP(:, :, limb_id) = graspable_points_in_World_frame - proj_point_j1_in_the_World_frame(:, limb_id);
      end

      % Vector from base position to first joint position projected on the terrain surface in the World frame
      proj_vec_base_to_j1 = first_joint_position_in_World_frame - proj_point_base_in_World_frame;
      % Angle from first joint angle 0 to graspable points
      angle_to_GP = NaN([size(graspable_points_in_World_frame), double(kNumLimb)]);

      GP_idx_in_j1_limit = false([1, size(graspable_points_in_World_frame, 2), double(kNumLimb)]);
      graspable_points_in_reachable_area_tmp_1 = NaN([size(graspable_points_in_World_frame), double(kNumLimb)]);
      for limb_id = 1 : kNumLimb
        angle_to_GP(1, :, limb_id) = acos((proj_vec_base_to_j1(:, limb_id)' * proj_vec_j1_to_GP(:, :, limb_id)) ./ (norm(proj_vec_base_to_j1(:, limb_id)) * vecnorm(proj_vec_j1_to_GP(:, :, limb_id))));

        GP_idx_in_j1_limit(1, :, limb_id) = angle_to_GP(1, :, limb_id) > kMinJointLimitRad(kJoints(1, limb_id), 1) & angle_to_GP(1, :, limb_id) < kMaxJointLimitRad(kJoints(1, limb_id), 1);
        graspable_points_in_reachable_area_tmp_1(:, GP_idx_in_j1_limit(1, :, limb_id), limb_id) = graspable_points_in_World_frame(:, GP_idx_in_j1_limit(1, :, limb_id));
      end

      % Reachable area boundary on the far side on the terrain surface for each limb in the World frame
      far_boundary_on_surface_in_World_frame = reachable_area.getFarBoundaryOnTerrainSurface();
      % Vector from first joint position to far side boundary projected on the terrain surface in the World frame
      proj_vec_j1_to_far_boundary = NaN(size(far_boundary_on_surface_in_World_frame));
      % Vector from first joint position to graspable points position in first joint limit projected on the terrain surface in the World frame
      proj_vec_j1_to_GP_in_j1_limit = NaN(size(graspable_points_in_World_frame));

      GP_idx_in_far_boundary = false([1, size(graspable_points_in_World_frame, 2), double(kNumLimb)]);
      for limb_id = 1 : kNumLimb
        proj_vec_j1_to_far_boundary(:, :, limb_id) = far_boundary_on_surface_in_World_frame(:, :, limb_id) - proj_point_j1_in_the_World_frame(:, limb_id);
        [max_dist_from_j1, ~] = max(vecnorm(proj_vec_j1_to_far_boundary(:, :, limb_id)));

        proj_vec_j1_to_GP_in_j1_limit(:, :, limb_id) = graspable_points_in_reachable_area_tmp_1(:, :, limb_id) - proj_point_j1_in_the_World_frame(:, limb_id);
        GP_idx_in_far_boundary(1, :, limb_id) = vecnorm(proj_vec_j1_to_GP_in_j1_limit(:, :, limb_id)) < max_dist_from_j1;
      end

      % Reachable area boundary on the near side on the terrain surface for each limb in the World frame
      near_boundary_on_surface_in_World_frame = reachable_area.getNearBoundaryOnTerrainSurface();
      % Vector from first joint position to near side boundary projected on the terrain surface in the World frame
      proj_vec_j1_to_near_boundary = NaN(size(near_boundary_on_surface_in_World_frame));

      GP_idx_in_near_boundary = false([1, size(graspable_points_in_World_frame, 2), double(kNumLimb)]);
      for limb_id = 1 : kNumLimb
        proj_vec_j1_to_near_boundary(:, :, limb_id) = near_boundary_on_surface_in_World_frame(:, :, limb_id) - proj_point_j1_in_the_World_frame(:, limb_id);
        [max_dist_from_j1, ~] = max(vecnorm(proj_vec_j1_to_near_boundary(:, :, limb_id)));
        GP_idx_in_near_boundary(1, :, limb_id) = vecnorm(proj_vec_j1_to_GP(:, :, limb_id)) < max_dist_from_j1;
      end

      GP_idx_in_boundary = (GP_idx_in_j1_limit & GP_idx_in_far_boundary) | GP_idx_in_near_boundary;
      graspable_points_in_reachable_area = NaN([size(graspable_points_in_World_frame), double(kNumLimb)]);
      for limb_id = 1 : kNumLimb
        graspable_points_in_reachable_area(:, GP_idx_in_boundary(1, :, limb_id), limb_id) = graspable_points_in_World_frame(:, GP_idx_in_boundary(1, :, limb_id));
        if (limb_id == 2)
          scatter3(graspable_points_in_reachable_area(1, :, limb_id), graspable_points_in_reachable_area(2, :, limb_id), graspable_points_in_reachable_area(3, :, limb_id), 40, "m", "filled");
        end
      end

      graspable_points.point_cloud_ = reshape(graspable_points_in_reachable_area, 3, []);
    end

  end

  %% Private Methods
  methods (Access = private)

    function createGraspablePointsGraphics(graspable_points)
      graspable_points.graphics_ = scatter3( ...
        graspable_points.point_cloud_(1, :), ...
        graspable_points.point_cloud_(2, :), ...
        graspable_points.point_cloud_(3, :), ...
        Marker = graspable_points.kMarkerStyle_, ...
        SizeData = graspable_points.kMarkerSize_, ...
        MarkerFaceColor = graspable_points.kColor_, ...
        MarkerEdgeColor = "none", ...
        MarkerFaceAlpha = graspable_points.kTransparency_, ...
        Visible = "off");
    end

  end

  %% Getter
  methods (Access = public)

    function kPointCloud = getPointCloud(graspable_points)
      kPointCloud = graspable_points.point_cloud_;
    end

    function nearest_point = getNearestPoint(graspable_points, original_position)
      arguments (Input)
        graspable_points;
        original_position (3, 1) {mustBeA(original_position, "double")};
      end

      [~, nearest_GPs_id] = min( ...
        (graspable_points.point_cloud_(1, :) - original_position(1, 1)) .^ 2 + ...
        (graspable_points.point_cloud_(2, :) - original_position(2, 1)) .^ 2 + ...
        (graspable_points.point_cloud_(3, :) - original_position(3, 1)) .^ 2);

      nearest_point = [ graspable_points.point_cloud_(1, nearest_GPs_id); ...
                        graspable_points.point_cloud_(2, nearest_GPs_id); ...
                        graspable_points.point_cloud_(3, nearest_GPs_id)];
    end

  end

end  % GraspablePoints
