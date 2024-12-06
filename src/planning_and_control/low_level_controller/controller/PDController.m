classdef PDController
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    Kp (1, 1) double;
    Kd (1, 1) double;
  end

  %% Public Methods
  methods (Access = public)

    function controller = PDController(proportional_gain, derivative_gain)
    % PDController() Constructor
      controller.Kp = proportional_gain;
      controller.Kd = derivative_gain;
    end

    function torque = calcJointTorque(controller, ...
        desired_angular_position, desired_angular_velocity, ...
        current_angular_position, current_angular_velocity)
      diff_position = desired_angular_position - current_angular_position;
      diff_velocity = desired_angular_velocity - current_angular_velocity;

      torque = controller.Kp * diff_position + controller.Kd * diff_velocity;
    end

  end

end
% EOF