classdef PDController < handle
% PDController
% PD controller
%
% Created     : 2020.04.10 by Warley Ribeiro
% Last updated: 2024.12.13 by Masazumi Imai

%% Properties
  properties (SetAccess = private, GetAccess = public)
    kProportionalGain_ (1, 1) double;
    kDerivativeGain_ (1, 1) double;
  end

  %% Public Methods
  methods (Access = public)

    function pd_controller = PDController(config_joint_controller)
    % PDController() Constructor
      arguments (Input)
        config_joint_controller (1, 1) {mustBeA(config_joint_controller, "ConfigJointController")};
      end

      [proportional_gain, derivative_gain] = config_joint_controller.getPDControllerGain();

      pd_controller.kProportionalGain_ = proportional_gain;
      pd_controller.kDerivativeGain_ = derivative_gain;
    end

    function torque = calcJointTorque(pd_controller, robot)
    % calcJointTorque()
    % Calculate joint torque
      arguments (Input)
        pd_controller;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      desired_angular_position = robot.des_SV_.getJointAngularPosition();
      desired_angular_velocity = robot.des_SV_.getJointAngularVelocity();

      current_angular_position = robot.SV_.getJointAngularPosition();
      current_angular_velocity = robot.SV_.getJointAngularVelocity();

      diff_position = desired_angular_position - current_angular_position;
      diff_velocity = desired_angular_velocity - current_angular_velocity;

      torque = pd_controller.kProportionalGain_ * diff_position + ...
               pd_controller.kDerivativeGain_ * diff_velocity;
    end

  end

end  % PDController
