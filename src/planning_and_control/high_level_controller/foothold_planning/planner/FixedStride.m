classdef FixedStride
% FixedStride
% Foothold planning method. Select the next swing limb numbers based on the periodic gait sequence
% and foothold positions based on the moving direction and graspable points.
%
% Created     : 2020.04.13 by Warley Ribeiro
% Last updated: 2024.12.07 by Masazumi Imai

  %% Public Methods
  methods (Access = public)

    function fixed_stride = FixedStride()
    % FixedStride() Constructor
    end

    function next_swing_limb_id = updateSwingLimbNumber(~, previous_swing_limb_id, gait_planning)
    % updateSwingLimbNumber()
    %   Update the next swing limb ID(s) based on the periodic gait sequence.
      arguments (Input)
        ~;
        previous_swing_limb_id (:, 1) {mustBeA(previous_swing_limb_id, "uint8")};
        gait_planning          (1, 1) {mustBeA(gait_planning,          "GaitPlanning")};
      end

      gait_sequence = gait_planning.scheduler_.getSequence();

      if (previous_swing_limb_id == 0 || ...  % Initial condition
          previous_swing_limb_id == gait_sequence(:, end))
        next_swing_limb_id = gait_sequence(:, 1);
      else
        [~, pre_swing_limb_id] = find(gait_sequence == previous_swing_limb_id, 1);
        next_swing_limb_id = gait_sequence(:, pre_swing_limb_id + 1);
      end
    end

    function next_foothold_positions = updateFootholdPositions(~, ...
        terrain, path_planning, foothold_planning)
    % updateFootholdPositions()
    %   Update next foothold positions based on moving direction and graspable points.
      arguments (Input)
        ~;
        terrain           (1, 1) {mustBeA(terrain,           "Terrain")};
        path_planning     (1, 1) {mustBeA(path_planning,     "PathPlanning")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
      end

      graspable_points = terrain.getGraspablePoints();
      moving_direction = path_planning.local_path_.getMovingDirection();
      swing_limb_id = foothold_planning.getSwingLimbID();
      current_foothold_positions = foothold_planning.getFootholdPositions();
      max_allowable_stride = foothold_planning.getMaxAllowableStride();

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
    end

  end

end  % FixedStride
