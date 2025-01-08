classdef ReachableArea < handle
% Reachable area
%
% Created     : 2020.05.13 by Yusuke Koizumi
% Last updated: 2025.01.08 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = private)
    kVisualizeReachableArea_ (1, 1) logical;
  end
  properties (SetAccess = private, GetAccess = public)
    % Position vectors of reachable boundary on the near side
    kBoundaryPositionVectorNear_
    % Position vectors of reachable boundary on the far side
    kBoundaryPositionVectorFar_
    % Minimum range from base CoM
    kMinRange_ (1, 1) double;
    % Maximum range from base CoM
    kMaxRange_ (1, 1) double;

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

      % Change the joint 2 when joint 3 bends up maximally
      d_theta_2 = 0.5;
      SV_tmp.q(1, 1) = deg2rad(45.0);
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
      SV_tmp.q(1, 1) = deg2rad(45.0);
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
      SV_tmp.q(1, 1) = deg2rad(45.0);
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
      SV_tmp.q(1, 1) = deg2rad(45.0);
      SV_tmp.q(2, 1) = deg2rad(kMinJointLimit(2, 1));
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

      % Position vectors of reachable boundary on the near side
      boundary_position_vector_near = [end_effector_position_tmp_2', end_effector_position_tmp_4'];
      % Position vectors of reachable boundary on the far side
      boundary_position_vector_far = [end_effector_position_tmp_3', end_effector_position_tmp_1'];

      % Index of minimum z position on the far side boundary
      [~, idx_min] = min(boundary_position_vector_far(3, :));

      reachable_area.kBoundaryPositionVectorNear_ = [fliplr(boundary_position_vector_far(:, 1 : idx_min)), boundary_position_vector_near];
      reachable_area.kBoundaryPositionVectorFar_ = [boundary_position_vector_far(:, idx_min : length(boundary_position_vector_far))];

      base_position = SV.getBasePosition();
      projection_point_of_base_position_in_world_frame = terrain.getProjectionPointInWorldFrame(base_position);
      vector_base_to_proj = projection_point_of_base_position_in_world_frame - base_position;
      % Find the points where z = ground out of dataset
      distance_near = abs(reachable_area.kBoundaryPositionVectorNear_(3, :) - vector_base_to_proj(3, 1));
      distance_far = abs(reachable_area.kBoundaryPositionVectorFar_(3, :) - vector_base_to_proj(3, 1));
      [~, idx_near] = min(distance_near(1, :));
      [~, idx_far] = min(distance_far(1, :));

      min_z_distance_near = reachable_area.kBoundaryPositionVectorNear_(2, idx_near(1, 1));
      min_z_distance_far = reachable_area.kBoundaryPositionVectorFar_(2, idx_far(1, 1));
      reachable_area.kMinRange_ = min_z_distance_near;
      reachable_area.kMaxRange_ = min_z_distance_far;
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
      num_joints_of_swing_limb = kNumJointsPerLimb(1, limb_id);
      Qi = LP.getRotationalRelationshipOfLinkFrames();
      base_orientation_euler = SV.getBaseOrientationEuler();
      % Angle from Base frame to first joint frame of limb in the World frame
      alpha = Qi(3, num_joints_of_swing_limb * (limb_id - 1) + 1) + base_orientation_euler(3, 1);

      c0 = LP.getPositionVectorFromBaseCoMToJoint();
      base_position = SV.getBasePosition();
      base_orientation_DCM = SV.getBaseOrientationDCM();
      % Position of the first joint of limb in the World frame
      first_joint_position = base_position + base_orientation_DCM * c0(:, num_joints_of_swing_limb * (limb_id - 1) + 1);

      % Minimum range of reachable area from the first joint of limb
      min_range_from_first_joint = reachable_area.kMinRange_ - c0(1, 1);
      % Maximum range of reachable area from the first joint of limb
      max_range_from_first_joint = reachable_area.kMaxRange_ - c0(1, 1);

      [kMinJointLimit, kMaxJointLimit] = LP.getJointLimit();
      first_joint_angle_range = linspace(deg2rad(kMinJointLimit(1, 1)), deg2rad(kMaxJointLimit(1, 1)), 10);

      % Arc in x-y plane of the World frame
      arc_min = [ min_range_from_first_joint * cos(first_joint_angle_range);
                  min_range_from_first_joint * sin(first_joint_angle_range);
                  zeros(1, length(first_joint_angle_range))];
      arc_max = [ max_range_from_first_joint * cos(-first_joint_angle_range);
                  max_range_from_first_joint * sin(-first_joint_angle_range);
                  zeros(1, length(first_joint_angle_range))];
      arc_min = rot_z(alpha) * arc_min;
      arc_max = rot_z(alpha) * arc_max;
      % Arc in x-y plane of the Ground frame
      kInclination = terrain.getSurfaceInclination();
      arc_min = rpy2dc(deg2rad(kInclination))' * arc_min;
      arc_max = rpy2dc(deg2rad(kInclination))' * arc_max;

      % Arc in the first joint frame
      projection_point_of_first_joint_position_in_world_frame = terrain.getProjectionPointInWorldFrame(first_joint_position);
      vector_joint_to_proj = projection_point_of_first_joint_position_in_world_frame - first_joint_position;
      arc_min = arc_min + first_joint_position + vector_joint_to_proj;
      arc_max = arc_max + first_joint_position + vector_joint_to_proj;
      % Offset from actual position so that reachable area should be drawn a bit higher not to be buried in the terrain surface visualization
      kOffset = 0.005;
      unit_normal_vec = -vector_joint_to_proj / norm(vector_joint_to_proj);
      arc_min = arc_min + kOffset * unit_normal_vec;
      arc_max = arc_max + kOffset * unit_normal_vec;

      boundary_on_surface = [arc_min, arc_max, arc_min];

      reachable_area.arc_endpoint_(:, :, 1) = arc_min(:, [1, end]);
      reachable_area.arc_endpoint_(:, :, 2) = arc_max(:, [1, end]);
    end

  end

  %% Getter
  methods (Access = public)

    % function [kBoundaryPositionVectorNear, kBoundaryPositionVectorFar] = getBoundaryPositionVector(reachable_area)
    %   kBoundaryPositionVectorNear = reachable_area.kBoundaryPositionVectorNear_;
    %   kBoundaryPositionVectorFar = reachable_area.kBoundaryPositionVectorFar_;
    % end

    % function [kMinRange, kMaxRange] = getReachableRange(reachable_area)
    %   kMinRange = reachable_area.kMinRange_;
    %   kMaxRange = reachable_area.kMaxRange_;
    % end

  end

end  % ReachableArea
