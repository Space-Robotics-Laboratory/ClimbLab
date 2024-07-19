classdef MotionPlanning

  properties (SetAccess = immutable, GetAccess = public)
    base_trajectory_type (1, 1) string;
    limb_trajectory_type (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    base_trajectory;
    limb_trajectory LimbEETrajectory;
  end

  methods (Access = public)
    % Constructor
    function motion_planning = MotionPlanning(base_trajectory_type, limb_trajectory_type, num_limb)
      arguments (Input)
        base_trajectory_type (1, 1) {mustBeA(base_trajectory_type, "string")};
        limb_trajectory_type (1, 1) {mustBeA(limb_trajectory_type, "string")};
        num_limb (1, 1) {mustBeA(num_limb, "uint8")};
      end
      motion_planning.base_trajectory_type = base_trajectory_type;
      motion_planning.limb_trajectory_type = limb_trajectory_type;

      for limb_id = 1:num_limb
        motion_planning.limb_trajectory(limb_id, 1) = LimbEETrajectory(limb_trajectory_type);
      end

      % TODO: Implement
      motion_planning.base_trajectory = [];
    end

    function motion_planning = initializeTrajectories(motion_planning, ...
        current_EE_position, line_style, color, width)
      num_limb = size(motion_planning.limb_trajectory, 1);
      for limb_id = 1:num_limb
        motion_planning.limb_trajectory(limb_id, 1) = ...
          motion_planning.limb_trajectory(limb_id, 1).initialize( ...
            current_EE_position(:, limb_id), line_style, color, width);
      end
    end

    function motion_planning = planTrajectories(motion_planning, ...
        motion_duration, current_EE_position, desired_EE_position, step_height)
      arguments (Input)
        motion_planning;
        motion_duration     (1, 1) {mustBeA(motion_duration,     "double")};
        current_EE_position (3, :) {mustBeA(current_EE_position, "double")};
        desired_EE_position (3, :) {mustBeA(desired_EE_position, "double")};
        step_height         (1, 1) {mustBeA(step_height,         "double")};
      end

      num_limb = size(motion_planning.limb_trajectory, 1);
      for limb_id = 1:num_limb
        motion_planning.limb_trajectory(limb_id, 1) = ...
          motion_planning.limb_trajectory(limb_id, 1).plan( ...
            motion_duration, current_EE_position(:, limb_id), desired_EE_position(:, limb_id), ...
            step_height);
      end
    end

    function motion_planning = updateForCurrentTimeStep(motion_planning, ...
        current_time, motion_duration)
      arguments (Input)
        motion_planning;
        current_time    (1, 1) {mustBeA(current_time, "double")};
        motion_duration (1, 1) {mustBeA(motion_duration, "double")};
      end

      num_limb = size(motion_planning.limb_trajectory, 1);
      for limb_id = 1:num_limb
        motion_planning.limb_trajectory(limb_id, 1) = ...
          motion_planning.limb_trajectory(limb_id, 1).update(current_time, motion_duration);
      end
    end
  end

end
% EOF