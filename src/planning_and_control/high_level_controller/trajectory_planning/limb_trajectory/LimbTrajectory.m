classdef LimbTrajectory < handle
% LimbTrajectory
% Plan limb end-effector pose trajectory and calculate desired pose at current time step
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kType_ (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    position_ PositionTrajectory;
    orientation_;
  end
  properties (Constant, GetAccess = private)
    kStartTime_         (1, 1) double = 0.0;              % [s]
    kStartVelocity_     (3, 1) double = [0.0; 0.0; 0.0];  % [m/s]
    kFinalVelocity_     (3, 1) double = [0.0; 0.0; 0.0];  % [m/s]
    kStartAcceleration_ (3, 1) double = [0.0; 0.0; 0.0];  % [m/s^2]
    kFinalAcceleration_ (3, 1) double = [0.0; 0.0; 0.0];  % [m/s^s]
  end
  properties (SetAccess = private, GetAccess = public)
    mid_time_     (1, 1) double;  % [s]
    mid_position_ (3, 1) double;  % [m]
    mid_velocity_ (3, 1) double;  % [m/s]
  end

  %% Methods called only from TrajectoryPlanning
  methods (Access = ?TrajectoryPlanning)

    function limb_trajectory = LimbTrajectory(config_trajectory_planning)
    % LimbTrajectory() Constructor
      arguments (Input)
        config_trajectory_planning (1, 1) {mustBeA(config_trajectory_planning, "ConfigTrajectoryPlanning")};
      end
      [~, limb_trajectory.kType_] = config_trajectory_planning.getTrajectoryType();

      limb_trajectory.position_ = PositionTrajectory(limb_trajectory.kType_);

      % TODO: Implement
      limb_trajectory.orientation_ = [];

      if (~config_trajectory_planning.getVisualizeLimbTrajectory())
        return;
      end

      [kLineStyle, kColor, kWidth] = config_trajectory_planning.getLimbTrajectoryVisualSettings();
      limb_trajectory.position_.planned_trajectory_.setVisualSettings(kLineStyle, kColor, kWidth);
    end

    function plan(limb_trajectory, robot, foothold_planning, gait_planning, limb_id)
    % plan()
    %   Plan the trajectory from current to desired pose of End-Effector
      arguments (Input)
        limb_trajectory;
        robot             (1, 1) {mustBeA(robot, "Robot")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
        limb_id           (1, 1) {mustBeA(limb_id, "uint8")};
      end

      current_EE_position = robot.getEEPosition(1:3, limb_id);  % Contact position?
      desired_EE_position = foothold_planning.planner_.output_.getFootholdPosition(1:3, limb_id);
      step_height = gait_planning.getStepHeight();

      motion_duration = gait_planning.scheduler_.output_.getTransferDuration();

      limb_trajectory.calcMidTime(motion_duration);
      limb_trajectory.calcMidPosition(current_EE_position, desired_EE_position, step_height);
      limb_trajectory.calcMidVelocity(motion_duration, current_EE_position, desired_EE_position);

      time_constraints         = [limb_trajectory.kStartTime_, ...
                                  limb_trajectory.mid_time_, ...
                                  motion_duration];

      position_constraints     = [current_EE_position, ...
                                  limb_trajectory.mid_position_, ...
                                  desired_EE_position];

      velocity_constraints     = [limb_trajectory.kStartVelocity_, ...
                                  limb_trajectory.mid_velocity_, ...
                                  limb_trajectory.kFinalVelocity_];

      acceleration_constraints = [limb_trajectory.kStartAcceleration_, ...
                                  limb_trajectory.kFinalAcceleration_];

      limb_trajectory.position_.plan(time_constraints, position_constraints, velocity_constraints, acceleration_constraints);

      limb_trajectory.position_.storePlannedTrajectory(limb_trajectory.kStartTime_, motion_duration);
    end

    function update(limb_trajectory, current_time, motion_start_time, motion_final_time)
    % update()
    %   Update End-Effector pose for current time step based on planned trajectory
      arguments (Input)
        limb_trajectory;
        current_time      (1, 1) {mustBeA(current_time,      "double")};
        motion_start_time (1, 1) {mustBeA(motion_start_time, "double")};
        motion_final_time (1, 1) {mustBeA(motion_final_time, "double")};
      end

      limb_trajectory.position_.update(current_time, motion_start_time, motion_final_time);
    end

    function stay(limb_trajectory, current_EE_position)
    % stay
    %   Return current End-Effector pose as desired pose
      arguments (Input)
        limb_trajectory;
        current_EE_position (3, 1) {mustBeA(current_EE_position, "double")};
      end

      limb_trajectory.position_.stay(current_EE_position);
    end

  end

  %% Private Methods
  methods (Access = private)

    function calcMidTime(limb_trajectory, final_time)
      limb_trajectory.mid_time_ = (limb_trajectory.kStartTime_ + final_time) / 2;
    end

    function calcMidPosition(limb_trajectory, start_position, final_position, step_height)
      % Vector from the current foot to the desired foothold positions
      vec_cur2des = final_position - start_position;
      % Projection of "vec_cur2des" on the x-y plane of World frame
      vec_cur2des_proj = [vec_cur2des(1 : 2, 1); 0.0];

      % pitch and yaw angle of the vector from current grasping point to the next grasping point
      % relative to the inertial frame
      if (norm(vec_cur2des_proj) == 0.0)
        if (vec_cur2des(3, 1) > 0)
          pitch = pi / 2.0;
        else
          pitch = - pi / 2.0;
        end
      else
        pitch = real( acos( ...
          dot(vec_cur2des, vec_cur2des_proj) / (norm(vec_cur2des) * norm(vec_cur2des_proj)) ) );
      end
      yaw = atan2( vec_cur2des_proj(2, 1), vec_cur2des_proj(1, 1) );
      I_R_S = rpy2dc( [0; pitch; yaw] );

      limb_trajectory.mid_position_ = start_position + vec_cur2des / 2.0 + ...
        I_R_S * [0.0; 0.0; step_height];
    end

    function calcMidVelocity(limb_trajectory, final_time, start_position, final_position)
      limb_trajectory.mid_velocity_ = ...
        2.0 * (final_position - start_position) / (final_time - limb_trajectory.kStartTime_);
    end

  end

end  % LimbTrajectory
