classdef TumbleStabilityMargin < handle

  properties (SetAccess = immutable, GetAccess = public)
  end
  properties (SetAccess = private, GetAccess = private)
    force_due_to_gravity_acceleration (3, 1);   % [N]
    moment_due_to_gravity_acceleration (3, 1);  % [Nm]

    force_due_to_inertial_acceleration (3, 1);   % [N]
    moment_due_to_inertial_acceleration (3, 1);  % [Nm]
  end

  methods (Access = public)
    % Constructor
    function this = TumbleStabilityMargin()
    end
  end

  methods (Access = private)

    % Obtain force and moment acting on the robot due to gravity acceleration at the CoM position
    function calcGravitationalForceMoment(this, gravity, mass, CoM)
      % Force acting due to gravity acceleration
      this.force_due_to_gravity_acceleration = mass * gravity;
      this.moment_due_to_gravity_acceleration = cross(CoM, this.force_due_to_gravity_acceleration);
    end

    % Obtain force and moment acting on the robot due to inertial acceleration, including angular
    % velocity and acceleration effects of each link
    function calcInertialForceMoment(this, base_mass, base_inertia, base_position, ...
        base_angular_velocity, base_linear_acceleration, base_angular_acceleration, ...
        num_joints, link_mass, link_inertia, link_position, link_angular_velocity, ...
        link_linear_acceleration, link_angular_acceleration)
      force_due_to_base_linear_accel = base_mass * base_linear_acceleration;

      moment_due_to_base_linear_accel = cross(base_mass * base_position, base_linear_acceleration);
      moment_due_to_base_angular_accel = base_inertia * base_angular_acceleration + ...
        cross(base_angular_velocity, base_inertia * base_angular_velocity);

      force_due_to_link_linear_accel   = zeros(num_joints, 1);
      moment_due_to_link_linear_accel  = zeros(num_joints, 1);
      moment_due_to_link_angular_accel = zeros(num_joints, 1);
      for i = 1:num_joints
        % TODO: Add (n, m)
        force_due_to_link_linear_accel(i, 1) = link_mass * link_linear_acceleration;

        moment_due_to_link_linear_accel(i, 1) = ...
          cross(link_mass * link_position, link_linear_acceleration);

        moment_due_to_link_angular_accel(i, 1) = link_inertia * link_angular_acceleration + ...
          cross(link_angular_velocity, link_inertia * link_angular_velocity);
      end

      this.force_due_to_inertial_acceleration = ...
        force_due_to_base_linear_accel + sum(force_due_to_link_linear_accel);

      this.moment_due_to_inertial_acceleration = ...
        moment_due_to_base_linear_accel + sum(moment_due_to_link_linear_accel) + ...
        moment_due_to_base_angular_accel + sum(moment_due_to_link_angular_accel);
    end

  end

end
% EOF