classdef FixedStride < handle
% FixedStride
% Foothold planning method. Select the next swing limb numbers based on the periodic gait sequence
% and foothold positions based on the moving direction and graspable points.
%
% Created     : 2020.04.13 by Warley Ribeiro
% Last updated: 2025.01.08 by Masazumi Imai

  properties (SetAccess = private, GetAccess = public)
    output_ FootholdPlannerOutput;
  end

  %% Public Methods
  methods (Access = public)

    function fixed_stride = FixedStride(kNumLimb)
    % FixedStride() Constructor
      arguments (Input)
        kNumLimb (1, 1) {mustBeA(kNumLimb, "uint8")};
      end

      fixed_stride.output_ = FootholdPlannerOutput(kNumLimb);
    end

    function updateSwingLimbId(fixed_stride, gait_planning)
    % updateSwingLimbId()
    %   Update the next swing limb ID(s) based on the periodic gait sequence.
      arguments (Input)
        fixed_stride;
        % previous_swing_limb_id (:, 1) {mustBeA(previous_swing_limb_id, "uint8")};
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      previous_swing_limb_id = fixed_stride.output_.getSwingLimbId();
      gait_sequence = gait_planning.scheduler_.getSequence();

      if (previous_swing_limb_id == 0 || ...  % Initial condition
          previous_swing_limb_id == gait_sequence(:, end))
        next_swing_limb_id = gait_sequence(:, 1);
      else
        [~, pre_swing_limb_id] = find(gait_sequence == previous_swing_limb_id, 1);
        next_swing_limb_id = gait_sequence(:, pre_swing_limb_id + 1);
      end

      % TODO: Create abstract class
      fixed_stride.output_.setSwingLimbId(next_swing_limb_id);

      fixed_stride.output_.setSwingLimbIdHistory(next_swing_limb_id);
    end

    function updateFootholdPositions(fixed_stride, terrain, path_planning, foothold_planning)
    % updateFootholdPositions()
    %   Update next foothold positions based on moving direction and graspable points.
      arguments (Input)
        fixed_stride;
        terrain           (1, 1) {mustBeA(terrain,           "Terrain")};
        path_planning     (1, 1) {mustBeA(path_planning,     "PathPlanning")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
      end

      graspable_points = terrain.getGraspablePoints();
      moving_direction = path_planning.local_path_.getMovingDirection();
      swing_limb_id = fixed_stride.output_.getSwingLimbId();
      current_foothold_positions = fixed_stride.output_.getFootholdPosition();
      max_allowable_stride = foothold_planning.getAllowableMaxStride();

      kNumLimb = uint8(size(current_foothold_positions, 2));
      ideal_next_EE_positions = zeros(3, kNumLimb);
      next_foothold_positions = zeros(3, kNumLimb);

      for limb_id = 1 : kNumLimb
        if (any(limb_id ~= swing_limb_id))
          next_foothold_positions(:, limb_id) = current_foothold_positions(:, limb_id);
          continue;
        end

        step_to_goal = moving_direction * max_allowable_stride;
        ideal_next_EE_positions(:, limb_id) = current_foothold_positions(:, limb_id) + step_to_goal;

        next_foothold_positions(:, limb_id) = graspable_points.getNearestPoint( ...
          ideal_next_EE_positions(:, limb_id));
      end

      % TODO: Create abstract class
      fixed_stride.output_.setFootholdPosition(next_foothold_positions);

      for limb_id = 1 : kNumLimb
        if (any(limb_id ~= swing_limb_id))
          continue;  % Do not update foothold history for support limb
        end

        fixed_stride.output_.setFootholdHistory(limb_id);
      end

    end

  end

  %% Getter
  methods (Access = public)

    function output = getOutput(planner)
      % TODO: Create abstract class and inherit from it like "Configuration" class
      output = planner.output_;
    end

  end

end  % FixedStride
