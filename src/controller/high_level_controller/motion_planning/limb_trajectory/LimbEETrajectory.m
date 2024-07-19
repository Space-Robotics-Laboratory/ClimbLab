classdef LimbEETrajectory

  properties (SetAccess = private, GetAccess = public)
    position EEPositionTrajectory;
    orientation;
  end

  methods (Access = ?MotionPlanning)
    % Constructor
    function limb_trajectory = LimbEETrajectory(trajectory_type)
      limb_trajectory.position = EEPositionTrajectory(trajectory_type);

      % TODO: Implement
      limb_trajectory.orientation = [];
    end

    function limb_trajectory = initialize(limb_trajectory, ...
        current_EE_position, line_style, color, width)
        limb_trajectory.position = limb_trajectory.position.initialize( ...
          current_EE_position, line_style, color, width);
    end

    function limb_trajectory = plan(limb_trajectory, ...
        motion_duration, current_EE_position, desired_EE_position, step_height)
      limb_trajectory.position = limb_trajectory.position.plan( ...
        motion_duration, current_EE_position, desired_EE_position, step_height);

      limb_trajectory.position = limb_trajectory.position.storePlannedTrajectory( ...
        motion_duration);
    end

    function limb_trajectory = update(limb_trajectory, current_time, motion_duration)
      limb_trajectory.position = limb_trajectory.position.update( ...
        current_time, motion_duration);
    end
  end

end
% EOF