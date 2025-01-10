classdef FootholdPlanning < handle
% FootholdPlanning
% Select the next swing limb numbers and foothold positions based on the planner and save these
% history
%
% Created     : 2020.04.13 by Warley Ribeiro
% Last updated: 2025.01.09 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kType_ (1, 1) string;
    planner_;

    kAllowableMaxStride_ (1, 1) double;  % TODO: Change name to kAllowableMaxStride_

    graspable_points_in_reachable_area_ GraspablePoints;
  end

  %% Public Methods
  methods (Access = public)

    function foothold_planning = FootholdPlanning(config_foothold_planning, robot)
    % FootholdPlanning() Constructor
      arguments (Input)
        config_foothold_planning (1, 1) {mustBeA(config_foothold_planning, "ConfigFootholdPlanning")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      kNumLimb = robot.LP_.getNumberOfLimb();
      current_EE_position = robot.getEEPosition();

      foothold_planning.kType_ = config_foothold_planning.getFootholdSelectionType();
      foothold_planning.setPlanner(kNumLimb);

      foothold_planning.kAllowableMaxStride_ = config_foothold_planning.getAllowableMaxStride();

      foothold_planning.planner_.output_.setFootholdPosition(current_EE_position);
      foothold_planning.planner_.output_.setFootholdHistory();

      foothold_planning.graspable_points_in_reachable_area_ = GraspablePoints();
    end

    function plan(foothold_planning, current_time, terrain, robot, perception, path_planning, gait_planning)
    % Plan the footholds based on the moving direction and graspable points.
      arguments (Input)
        foothold_planning;
        current_time  (1, 1) {mustBeA(current_time,  "double")};
        terrain       (1, 1) {mustBeA(terrain,       "Terrain")};
        robot         (1, 1) {mustBeA(robot,         "Robot")};
        perception    (1, 1) {mustBeA(perception,    "Perception")};
        path_planning (1, 1) {mustBeA(path_planning, "PathPlanning")};
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      if (foothold_planning.kType_ == "do_nothing")
        return;
      end

      if (~foothold_planning.isUpdateTiming(current_time, robot, gait_planning))
        return;  % Do not update if current time is during motion
      end

      robot.getKinematics().getReachableArea().updateBoundary(terrain, robot.getLinkParameter(), robot.getStateVariable());

      foothold_planning.graspable_points_in_reachable_area_.updateGraspablePointsInReachableArea(terrain, robot, perception);

      foothold_planning.planner_.plan(robot, path_planning, ...
        foothold_planning.graspable_points_in_reachable_area_, foothold_planning.kAllowableMaxStride_);

      % TODO: Change following 2 functions to foothold_planning.planner_.plan()
      % % Update swing limb ID and its history
      % foothold_planning.planner_.updateSwingLimbId(gait_planning);

      % % Update foothold position and its history
      % foothold_planning.planner_.updateFootholdPositions(terrain, path_planning, foothold_planning);
    end

  end

  %% Private Methods
  methods (Access = private)

    function is_update_timing = isUpdateTiming(foothold_planning, current_time, robot, gait_planning)
      arguments (Input)
        foothold_planning;
        current_time  (1, 1) {mustBeA(current_time,  "double")};
        robot         (1, 1) {mustBeA(robot,         "Robot")};
        gait_planning (1, :) {mustBeA(gait_planning, "GaitPlanning")};
      end

      is_update_timing = false;

      if (current_time == 0.0)  % Initial condition
        is_update_timing = true;
        return;
      end

      landing_time = gait_planning.getScheduler().getOutput().getLandingTimings();  % [s] (1 x kNumLimb)
      swing_limb_id = foothold_planning.planner_.getOutput().getSwingLimbId();
      is_swing_limb_landing_time = all(current_time == landing_time(1, swing_limb_id));

      is_all_limb_grasping = all(robot.getStateVariable().getIsGrasping());

      if (is_swing_limb_landing_time && is_all_limb_grasping)
        is_update_timing = true;
      end
    end

  end

  %% Setter
  methods (Access = private)

    function setPlanner(foothold_planning, kNumLimb)
      switch (foothold_planning.kType_)
        case "do_nothing"
          planner = [];
        case "fixed_stride"
          planner = FixedStride(kNumLimb);
        case "max_stride_to_goal_in_reachable_area"
          planner = MaxStrideToGoalInReachableArea(kNumLimb);
        otherwise
          error("Invalid foothold selection type is specified!!");
      end
      foothold_planning.planner_ = planner;
    end

  end

  %% Getter
  methods (Access = public)

    function kAllowableMaxStride = getAllowableMaxStride(foothold_planning)
      kAllowableMaxStride = foothold_planning.kAllowableMaxStride_;
    end

    function planner = getPlanner(foothold_planning)
      planner = foothold_planning.planner_;
    end

  end

end  % FootholdPlanning
