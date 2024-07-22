classdef FixedStride

  properties (SetAccess = private, GetAccess = public)
    swing_limb_number (:, 1) uint8;
    swing_limb_number_history uint8;

    foothold_positions (3, :) double;
    footholds_history (:, 1) Trajectory;

    sequence uint8;
      % 1st dim: Limb number(s) starting at same timing during gait cycle
      % 2nd dim: Limb number(s) starting at same different during gait cycle
    step_length (1, 1) double;
  end

  methods (Access = ?FootholdPlanning)
    % Constructor
    function fixed_stride = FixedStride(num_limb)
      arguments (Input)
        num_limb (1, 1) {mustBeA(num_limb, "uint8")};
      end
      fixed_stride.swing_limb_number = uint8(0);
      fixed_stride.swing_limb_number_history = [];
      for limb_id = 1:num_limb
        fixed_stride.foothold_positions(1:3, limb_id) = [0.0; 0.0; 0.0];
        fixed_stride.footholds_history(limb_id, 1) = Trajectory();
      end
      fixed_stride.sequence = uint8(0);
      fixed_stride.step_length = 0.0;
    end

    function fixed_stride = initialize(fixed_stride, sequence, step_length, current_EE_position)
      arguments (Input)
        fixed_stride;
        sequence
        step_length         (1, 1) {mustBeA(step_length,         "double")};
        current_EE_position (3, :) {mustBeA(current_EE_position, "double")};
      end
      fixed_stride.sequence = sequence;
      fixed_stride.swing_limb_number = fixed_stride.sequence(:, 1);
      fixed_stride.swing_limb_number_history = fixed_stride.swing_limb_number;

      fixed_stride.step_length = step_length;

      num_limb = size(fixed_stride.foothold_positions, 2);
      for limb_id = 1:num_limb
        fixed_stride.foothold_positions(:, limb_id) = current_EE_position(:, limb_id);
        fixed_stride.footholds_history(limb_id, 1) = ...
          fixed_stride.footholds_history(limb_id, 1).initialize( ...
            fixed_stride.foothold_positions(:, limb_id));
      end
    end

    function fixed_stride = plan(fixed_stride, ...
        current_time, moving_direction_unit_vector, graspable_points)
      arguments (Input)
        fixed_stride;
        current_time
        moving_direction_unit_vector (3, 1) {mustBeA(moving_direction_unit_vector, "double")};
        graspable_points             (1, 1) {mustBeA(graspable_points,    "GraspablePoints")};
      end

      if current_time ~= 0.0
        fixed_stride.swing_limb_number = fixed_stride.updateSwingLimbNumber();
        fixed_stride.swing_limb_number_history = fixed_stride.updateSwingLimbNumberHistory();
      end

      num_limb = uint8(size(fixed_stride.foothold_positions, 2));
      for limb_id = 1:num_limb
        if any(limb_id ~= fixed_stride.swing_limb_number)
          continue;
        end
        fixed_stride.foothold_positions(:, limb_id) = fixed_stride.updateFootholdPosition( ...
          limb_id, moving_direction_unit_vector, graspable_points);
        fixed_stride.footholds_history(limb_id, 1) = fixed_stride.updateFootholdsHistory(limb_id);
      end
    end
  end  % methods (Access = ?FootholdPlanning)

  methods (Access = private)
    function swing_limb_number =  updateSwingLimbNumber(fixed_stride)
      previous_swing_limb_number = fixed_stride.swing_limb_number;
      if previous_swing_limb_number == fixed_stride.sequence(:, end)
        swing_limb_number = fixed_stride.sequence(:, 1);
      else
        [~, pre_swing_limb_index] = find(fixed_stride.sequence == previous_swing_limb_number, 1);
        swing_limb_number = fixed_stride.sequence(:, pre_swing_limb_index + 1);
      end
    end

    function swing_limb_number_history = updateSwingLimbNumberHistory(fixed_stride)
      swing_limb_number_history = ...
        horzcat(fixed_stride.swing_limb_number_history, fixed_stride.swing_limb_number);
    end

    function next_foothold_position = updateFootholdPosition(fixed_stride, ...
      swing_limb_id, moving_direction_unit_vector, graspable_points)

      step_to_goal = moving_direction_unit_vector * fixed_stride.step_length;
      current_foothold_position = fixed_stride.foothold_positions(:, swing_limb_id);
      ideal_next_EE_position = current_foothold_position + step_to_goal;

      next_foothold_position = graspable_points.getNearestPoint(ideal_next_EE_position);
    end

    function footholds_history = updateFootholdsHistory(fixed_stride, swing_limb_id)
      foothold_position = fixed_stride.foothold_positions(:, swing_limb_id);
      footholds_history = ...
        fixed_stride.footholds_history(swing_limb_id, 1).addPoint(foothold_position);
    end
  end  % methods (Access = private)

end
% EOF