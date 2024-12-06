classdef MotionPlanning
  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    base_trajectory_type (1, 1) string;
    limb_trajectory_type (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    base_trajectory BaseTrajectory;
    limb_trajectory LimbTrajectory;
  end

  %% Public Methods
  methods (Access = public)

    function motion_planning = MotionPlanning(config, robot)
    % MotionPlanning() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigMotionPlanning")};
        robot  (1, 1) {mustBeA(robot, "Robot")};
      end

      num_limb = robot.LP.getNumberOfLimb();
      [motion_planning.base_trajectory_type, motion_planning.limb_trajectory_type] = ...
        config.getTrajectoryType();

      motion_planning.base_trajectory = BaseTrajectory(config);

      for limb_id = 1 : num_limb
        motion_planning.limb_trajectory(limb_id, 1) = LimbTrajectory(config);
      end
    end

    function motion_planning = plan(motion_planning, ...
        current_time, robot, foothold_planning, gait_planning)
    % plan()
    %   Plan trajectories from current to desired pose of base and End-Effectors, and
    %   update desired pose for the current time step of both
      arguments (Input)
        motion_planning;
        current_time      (1, 1) {mustBeA(current_time, "double")};
        robot             (1, 1) {mustBeA(robot, "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      swing_limb_id = foothold_planning.getSwingLimbID();
      swing_time = gait_planning.getSwingTimings();
      landing_time = gait_planning.getLandingTimings();
      if (current_time == 0.0 || any(current_time == swing_time(1, swing_limb_id)))
        motion_planning = motion_planning.planTrajectories(robot, foothold_planning, gait_planning);
      end

      contact_EE_positions = robot.contact_state.getPosition();
      motion_start_time = swing_time(1, swing_limb_id);
      motion_final_time = landing_time(1, swing_limb_id);

      motion_planning = motion_planning.updateForCurrentTimeStep( ...
        current_time, contact_EE_positions, swing_limb_id, motion_start_time, motion_final_time);
    end

    function motion_planning = visualize(motion_planning, time)
    % visualize()
    %   Visualize trajectories of each End-Effector
      arguments (Input)
        motion_planning;
        time (1, 1) {mustBeA(time, "double")};
      end

      if (time ~= 0.0)
        return;
      end
      num_limb = length(motion_planning.limb_trajectory);
      for limb_id = 1 : num_limb
        motion_planning.limb_trajectory(limb_id, 1).position.planned_trajectory.visualize();
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function motion_planning = planTrajectories(motion_planning, ...
        robot, foothold_planning, gait_planning)
      arguments (Input)
        motion_planning;
        robot             (1, 1) {mustBeA(robot, "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      motion_planning.base_trajectory = motion_planning.base_trajectory.plan(robot, gait_planning);

      num_limb = uint8(size(motion_planning.limb_trajectory, 1));
      swing_limb_id = foothold_planning.getSwingLimbID();
      for limb_id = 1 : num_limb
        if (all(limb_id ~= swing_limb_id))
          continue;
        end
        motion_planning.limb_trajectory(limb_id, 1) = ...
          motion_planning.limb_trajectory(limb_id, 1).plan( ...
            robot, foothold_planning, gait_planning, limb_id);
      end
    end

    function motion_planning = updateForCurrentTimeStep(motion_planning, ...
        current_time, contact_EE_positions, swing_limb_id, motion_start_time, motion_final_time)
      arguments (Input)
        motion_planning;
        current_time         (1, 1) {mustBeA(current_time, "double")};
        contact_EE_positions (3, :) {mustBeA(contact_EE_positions, "double")};
        swing_limb_id        (:, 1) {mustBeA(swing_limb_id, "uint8")};
        motion_start_time    (1, 1) {mustBeA(motion_start_time, "double")};
        motion_final_time    (1, 1) {mustBeA(motion_final_time, "double")};
      end

      motion_planning.base_trajectory = motion_planning.base_trajectory.update( ...
        current_time, motion_start_time, motion_final_time);

      num_limb = size(motion_planning.limb_trajectory, 1);
      for limb_id = 1 : num_limb
        if (any(limb_id == swing_limb_id))
          motion_planning.limb_trajectory(limb_id, 1) = ...
            motion_planning.limb_trajectory(limb_id, 1).update(current_time, motion_start_time, ...
            motion_final_time);
        else
          motion_planning.limb_trajectory(limb_id, 1) = ...
            motion_planning.limb_trajectory(limb_id, 1).stay(contact_EE_positions(:, limb_id));
        end
      end
    end

  end

  %% Getter
  methods (Access = public)
    function desired_base_position = getDesiredBasePosition(motion_planning)
      desired_base_position = motion_planning.base_trajectory.position.getDesiredPosition();
    end
    function desired_EE_positions = getDesiredEEPositions(motion_planning)
      num_limb = length(motion_planning.limb_trajectory);
      desired_EE_positions = zeros(3, num_limb);
      for limb_id = 1 : num_limb
        desired_EE_positions(:, limb_id) = ...
          motion_planning.limb_trajectory(limb_id, 1).position.getDesiredPosition();
      end
    end
  end

end
% EOF