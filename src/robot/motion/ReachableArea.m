classdef ReachableArea < handle
% Reachable area
%
% Created     : 2020.05.13 by Yusuke Koizumi
% Last updated: 2025.01.09 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = private)
    kVisualizeReachableArea_ (1, 1) logical;
  end
  properties (SetAccess = private, GetAccess = public)
    % Position vectors of reachable area boundary on the near side of "limb 1" in z direction of the Base frame
    % (when the robot base pose is same as origin of the World frame and first joint angle is 0)
    %   1st dim: x-y-z coordinates
    %   2nd dim: Number of points of boundary on the near side
    kNearBoundaryInZDirOfBaseFrame_ (3, :) double;
    % Position vectors of reachable area boundary on the far side of "limb 1" in z direction of the Base frame
    % (when the robot base pose is same as origin of the World frame and first joint angle is 0)
    %   1st dim: x-y-z coordinates
    %   2nd dim: Number of points of boundary on the near side
    kFarBoundaryInZDirOfBaseFrame_ (3, :) double;

    % Reachable area boundary on the terrain surface
    %   1st dim: x-y-z coordinates
    %   2nd dim: Number of points of boundary
    %   3rd dim: Limb ID
    boundary_on_surface_ (3, :, :) double;

    % Endpoint of arc on the near/far side within boundary on the terrain surface
    %   1st dim: x-y-z coordinates
    %   2nd dim: Number of endpoints of arc
    %   3rd dim: Near and far side
    arc_endpoint_ (3, 2, 2) double;
  end
  properties (SetAccess = private, GetAccess = private)
    graphics_ (:, 1) matlab.graphics.chart.primitive.Line;
  end

  %% Public Methods
  methods (Access = public)

    function reachable_area = ReachableArea(config_robot, terrain, LP, SV)
    % Constructor
      arguments (Input)
        config_robot (1, 1) {mustBeA(config_robot, "ConfigRobot")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
      end

      kNumLimb = LP.getNumberOfLimb();

      [reachable_area.kVisualizeReachableArea_, kLineColor, kLineWidth] = config_robot.getReachableAreaVisualSettings();
      if (reachable_area.kVisualizeReachableArea_)
        reachable_area.createReachableAreaGraphics(kNumLimb, kLineColor, kLineWidth);
      end

      reachable_area.calcReachableAreaForInsectJointConfig3DofLimb(terrain, LP, SV);

      % Reachable area boundary on the terrain surface for each limb
      for limb_id = 1 : kNumLimb
        boundary_on_surface(:, :, limb_id) = reachable_area.calcBoundary(terrain, LP, SV, limb_id);
      end
      reachable_area.boundary_on_surface_ = boundary_on_surface;
    end

    function updateBoundary(reachable_area, terrain, LP, SV)
      % Reachable area boundary on the terrain surface for each limb
      for limb_id = 1 : LP.getNumberOfLimb()
        reachable_area.boundary_on_surface_(:, :, limb_id) = reachable_area.calcBoundary(terrain, LP, SV, limb_id);
      end
    end

    function visualize(reachable_area, robot, foothold_planning)
      arguments (Input)
        reachable_area;
        robot             (1, 1) {mustBeA(robot,             "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
      end

      swing_limb_id = foothold_planning.getPlanner().getOutput().getSwingLimbId();

      for limb_id = 1 : robot.getLinkParameter().getNumberOfLimb()
        reachable_area.graphics_(limb_id, 1).XData = reachable_area.boundary_on_surface_(1, :, limb_id);
        reachable_area.graphics_(limb_id, 1).YData = reachable_area.boundary_on_surface_(2, :, limb_id);
        reachable_area.graphics_(limb_id, 1).ZData = reachable_area.boundary_on_surface_(3, :, limb_id);

        if (limb_id == swing_limb_id)
          reachable_area.graphics_(limb_id, 1).Visible = "on";
        else
          reachable_area.graphics_(limb_id, 1).Visible = "off";
        end
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function calcReachableAreaForInsectJointConfig3DofLimb(reachable_area, terrain, LP, SV)
    % Calculate reachable area for a 3 DoF limb that has a insect joint configuration
      arguments (Input)
        reachable_area;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
      end

      kNumJoints = LP.getNumberOfJoints();
      [kMinJointLimit, kMaxJointLimit] = LP.getJointLimit();

      LP_tmp = LP.clone();
      SV_tmp = SV.clone();
      % Set initial base pose
      SV_tmp.R0 = zeros(3, 1);
      SV_tmp.Q0 = zeros(3, 1);
      SV_tmp.A0 = rpy2dc(SV_tmp.Q0)';
      % Set initial joint angle
      SV_tmp.q = zeros(kNumJoints, 1);

      % Joint angle of first joint is fixed to 0 [deg]
      theta_1 = 0.0;

      % TODO: Need to consider all reachable area if need. The following 4 for loop is not enough
      % Change the joint 2 when joint 3 bends up maximally
      d_theta_2 = 0.5;
      SV_tmp.q(1, 1) = deg2rad(theta_1);
      SV_tmp.q(3, 1) = deg2rad(kMaxJointLimit(3, 1));
      range_theta_2 = kMinJointLimit(2, 1) : d_theta_2 : kMaxJointLimit(2, 1);
      end_effector_position_tmp_1 = NaN(numel(range_theta_2), 3);
      cnt = 1;
      for theta_2 = range_theta_2
        SV_tmp.q(2, 1) = deg2rad(theta_2);
        SV_tmp = calc_aa(LP_tmp, SV_tmp);
        SV_tmp = calc_pos(LP_tmp, SV_tmp);
        [end_effector_position, ~] = f_kin_e(LP_tmp, SV_tmp, 3);
        end_effector_position_tmp_1(cnt, :) = end_effector_position';
        cnt = cnt + 1;
      end

      % Change the joint 2 when joint 3 bends down minimally
      SV_tmp.q(1, 1) = deg2rad(theta_1);
      SV_tmp.q(3, 1) = deg2rad(kMinJointLimit(3, 1));
      end_effector_position_tmp_2 = NaN(numel(range_theta_2), 3);
      cnt = 1;
      for theta_2 = range_theta_2
        SV_tmp.q(2, 1) = deg2rad(theta_2);
        SV_tmp = calc_aa(LP_tmp, SV_tmp);
        SV_tmp = calc_pos(LP_tmp, SV_tmp);
        [end_effector_position, ~] = f_kin_e(LP_tmp, SV_tmp, 3);
        end_effector_position_tmp_2(cnt, :) = end_effector_position';
        cnt = cnt + 1;
      end

      % Change the joint 3 when joint 2 bends down minimally
      d_theta_3 = d_theta_2;
      SV_tmp.q(1, 1) = deg2rad(theta_1);
      SV_tmp.q(2, 1) = deg2rad(kMinJointLimit(2, 1));
      range_theta_3 = kMinJointLimit(3, 1) : d_theta_3 : kMaxJointLimit(3, 1);
      end_effector_position_tmp_3 = NaN(numel(range_theta_3), 3);
      cnt = 1;
      for theta_3 = range_theta_3
        SV_tmp.q(3, 1) = deg2rad(theta_3);
        SV_tmp = calc_aa(LP_tmp, SV_tmp);
        SV_tmp = calc_pos(LP_tmp, SV_tmp);
        [end_effector_position, ~] = f_kin_e(LP_tmp, SV_tmp, 3);
        end_effector_position_tmp_3(cnt, :) = end_effector_position';
        cnt = cnt + 1;
      end

      % Change the joint 3 when joint 2 bends up maximally
      SV_tmp.q(1, 1) = deg2rad(theta_1);
      SV_tmp.q(2, 1) = deg2rad(kMaxJointLimit(2, 1));
      end_effector_position_tmp_4 = NaN(numel(range_theta_3), 3);
      cnt = 1;
      for theta_3 = range_theta_3
        SV_tmp.q(3, 1) = deg2rad(theta_3);
        SV_tmp = calc_aa(LP_tmp, SV_tmp);
        SV_tmp = calc_pos(LP_tmp, SV_tmp);
        [end_effector_position, ~] = f_kin_e(LP_tmp, SV_tmp, 3);
        end_effector_position_tmp_4(cnt, :) = end_effector_position';
        cnt = cnt + 1;
      end

      %%% Calculation of reachable area boundary on the near/far side in the first joint frame
      position_vector = [end_effector_position_tmp_1', end_effector_position_tmp_2', end_effector_position_tmp_3', end_effector_position_tmp_4'];  % 3 x n matrix
      % Index of minimum y position on the position vector
      [~, idx_y_min] = min(position_vector(2, :));
      min_y_point = position_vector(2, idx_y_min);
      % Index of minimum z position on the position vector
      [~, idx_z_min] = min(position_vector(3, :));
      % Index of maximum z position on the position vector
      [~, idx_z_max] = max(position_vector(3, :));
      kDeltaZ = 0.005;
      kZRange = position_vector(3, idx_z_min) : kDeltaZ : position_vector(3, idx_z_max) - kDeltaZ;
      boundary_near_side = NaN(3, length(kZRange));
      boundary_far_side = NaN(3, length(kZRange));
      boundary_near_side(:, 1) = position_vector(:, idx_z_min);
      boundary_far_side(:, 1) = position_vector(:, idx_z_min);
      cnt = 2;
      for z_point = kZRange  % z_point: z position of point for getting boundary of reachable area
        % Index for points with z distance from z_point is small
        idx_small_z_dist = position_vector(3, :) > z_point & position_vector(3, :) <= z_point + kDeltaZ;
        points_with_small_z_dist = position_vector(:, idx_small_z_dist);
        % Index of point for boundary on the near side
        [~, idx_near] = min(points_with_small_z_dist(2, :) - min_y_point);
        boundary_near_side(:, cnt) = points_with_small_z_dist(:, idx_near);
        % Index of point for boundary on the far side
        [~, idx_far] = max(points_with_small_z_dist(2, :) - min_y_point);
        boundary_far_side(:, cnt) = points_with_small_z_dist(:, idx_far);
        cnt = cnt + 1;
      end
      boundary_near_side(:, end) = position_vector(:, idx_z_max);
      boundary_far_side(:, end) = position_vector(:, idx_z_max);

      reachable_area.kNearBoundaryInZDirOfBaseFrame_ = boundary_near_side;
      reachable_area.kFarBoundaryInZDirOfBaseFrame_ = boundary_far_side;
    end

    function createReachableAreaGraphics(reachable_area, kNumLimb, kLineColor, kLineWidth)
      arguments (Input)
        reachable_area;
        kNumLimb   (1, 1) {mustBeA(kNumLimb,   "uint8")};
        kLineColor        {mustBeA(kLineColor, ["double", "string"])};
        kLineWidth (1, 1) {mustBeA(kLineWidth, "double")};
      end

      boundary = zeros(3, 1);  % for initialization
      graphics_reachable_area = matlab.graphics.chart.primitive.Line.empty;
      for limb_id = 1 : kNumLimb
        graphics_reachable_area(limb_id, 1) = plot3(boundary(1, :), boundary(2, :), boundary(3, :), ...
          Color = kLineColor, LineWidth = kLineWidth, Visible = "off");
      end
      reachable_area.graphics_ = graphics_reachable_area;
    end

    function boundary_on_surface = calcBoundary(reachable_area, terrain, LP, SV, limb_id)
    % Calculation of reachable area boundary on the terrain surface for a limb
      arguments (Input)
        reachable_area;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
        LP      (1, 1) {mustBeA(LP,      "LinkParameters")};
        SV      (1, 1) {mustBeA(SV,      "StateVariable")};
        limb_id (1, 1) {mustBeA(limb_id, "uint8")};
      end

      kNumJointsPerLimb = LP.getNumberOfJointsPerLimb();
      num_joints_of_limb = kNumJointsPerLimb(1, limb_id);
      Qi = LP.getRotationalRelationshipOfLinkFrames();
      % Angle from Base frame to first joint frame of limb in the Base frame [rad]
      alpha = Qi(3, num_joints_of_limb * (limb_id - 1) + 1);

      c0 = LP.getPositionVectorFromBaseCoMToJoint();
      base_position = SV.getBasePosition();
      base_orientation_DCM = SV.getBaseOrientationDCM();
      % Position of the first joint of limb in the World frame
      first_joint_position = base_position + base_orientation_DCM * c0(:, num_joints_of_limb * (limb_id - 1) + 1);

      % Number of points forming reachable area boundary on the terrain surface
      kNumPointsOnSurface = 10;
      [kMinJointLimit, kMaxJointLimit] = LP.getJointLimit();
      first_joint_angle_range = linspace(deg2rad(kMinJointLimit(1, 1)), deg2rad(kMaxJointLimit(1, 1)), kNumPointsOnSurface);  % [rad]

      % Angle from Base frame to first joint frame of "limb 1" in the Base frame [rad]
      alpha_1 = Qi(3, 1);
      % Angle from first joint frame of "limb 1" to first joint frame of limb in the Base frame [rad]
      beta = alpha - alpha_1;
      % Position of the first joint (joint 1) of "limb 1" in the Base frame
      joint_1_position_in_Base_frame = c0(:, 1);
      % Boundary in z direction in the joint 1 frame
      near_boundary_in_z_dir = reachable_area.kNearBoundaryInZDirOfBaseFrame_ - joint_1_position_in_Base_frame;
      far_boundary_in_z_dir = reachable_area.kFarBoundaryInZDirOfBaseFrame_ - joint_1_position_in_Base_frame;
      % Boundary in z direction for each first joint angle in the joint 1 frame
      near_boundary_in_z_dir_for_each_angle = zeros(3, length(near_boundary_in_z_dir), length(first_joint_angle_range));
      far_boundary_in_z_dir_for_each_angle = zeros(3, length(far_boundary_in_z_dir), length(first_joint_angle_range));
      % Boundary in z direction for limb in the World frame
      near_boundary_in_z_dir_for_limb = zeros(size(near_boundary_in_z_dir_for_each_angle));
      far_boundary_in_z_dir_for_limb = zeros(size(far_boundary_in_z_dir_for_each_angle));
      cnt = 1;
      for theta_1 = beta + first_joint_angle_range
        near_boundary_in_z_dir_for_each_angle(:, :, cnt) = rot_z(theta_1) * near_boundary_in_z_dir;
        far_boundary_in_z_dir_for_each_angle(:, :, cnt) = rot_z(theta_1) * far_boundary_in_z_dir;

        near_boundary_in_z_dir_for_limb(:, :, cnt) = base_orientation_DCM * near_boundary_in_z_dir_for_each_angle(:, :, cnt) + first_joint_position;
        far_boundary_in_z_dir_for_limb(:, :, cnt) = base_orientation_DCM * far_boundary_in_z_dir_for_each_angle(:, :, cnt) + first_joint_position;
        cnt = cnt + 1;
      end

      % Calculation of reachable area boundary on the terrain surface
      arc_near_on_surface = zeros(3, kNumPointsOnSurface);
      arc_far_on_surface = zeros(3, kNumPointsOnSurface);
      for boundary_id = 1 : kNumPointsOnSurface
        % Projection points of boundary in z direction for limb in the World frame
        projection_points_of_near_boundary_in_z_dir = terrain.getProjectionPointInWorldFrameInZDirOfGroundFrame(near_boundary_in_z_dir_for_limb(:, :, boundary_id));
        projection_points_of_far_boundary_in_z_dir = terrain.getProjectionPointInWorldFrameInZDirOfGroundFrame(far_boundary_in_z_dir_for_limb(:, :, boundary_id));
        % Distance between before and after projection
        distance_near_boundary_points = vecnorm(near_boundary_in_z_dir_for_limb(:, :, boundary_id) - projection_points_of_near_boundary_in_z_dir, 2);
        distance_far_boundary_points = vecnorm(far_boundary_in_z_dir_for_limb(:, :, boundary_id) - projection_points_of_far_boundary_in_z_dir, 2);
        % Index of minimum distance
        [~, idx_min_dist_near] = min(distance_near_boundary_points);
        [~, idx_min_dist_far] = min(distance_far_boundary_points);
        arc_near_on_surface(:, boundary_id) = near_boundary_in_z_dir_for_limb(:, idx_min_dist_near, boundary_id);
        arc_far_on_surface(:, boundary_id) = far_boundary_in_z_dir_for_limb(:, idx_min_dist_far, boundary_id);
      end
      boundary_on_surface = [arc_near_on_surface, arc_far_on_surface, arc_near_on_surface(:, 1)];

      reachable_area.arc_endpoint_(:, :, 1) = arc_near_on_surface(:, [1, end]);
      reachable_area.arc_endpoint_(:, :, 2) = arc_far_on_surface(:, [1, end]);
    end

  end

  %% Getter
  methods (Access = public)

    function [kNearBoundaryInZDirOfBaseFrame, kFarBoundaryInZDirOfBaseFrame] = getBoundaryInZDirOfBaseFrame(reachable_area)
      kNearBoundaryInZDirOfBaseFrame = reachable_area.kNearBoundaryInZDirOfBaseFrame_;
      kFarBoundaryInZDirOfBaseFrame = reachable_area.kFarBoundaryInZDirOfBaseFrame_;
    end

  end

end  % ReachableArea
