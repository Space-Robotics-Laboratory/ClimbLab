classdef JointController
  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    type (1, 1) string;
    controller;
  end

  %% Public Methods
  methods (Access = public)

    function joint_controller = JointController(config)
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigJointController")};
      end

      joint_controller.type = config.getType();
      joint_controller.controller = joint_controller.setController(config);
    end

    function robot = control(joint_controller, robot)
      desired_angular_position = robot.des_SV.getJointAngularPosition();
      desired_angular_velocity = robot.des_SV.getJointAngularVelocity();
      current_angular_position = robot.SV.getJointAngularPosition();
      current_angular_velocity = robot.SV.getJointAngularVelocity();

      joint_torque = joint_controller.controller.calcJointTorque( ...
        desired_angular_position, desired_angular_velocity, ...
        current_angular_position, current_angular_velocity);
      robot = robot.setJointTorque(joint_torque);
    end

  end

  %% Private Methods
  methods (Access = private)

    function controller = setController(joint_controller, config)
      switch (joint_controller.type)
        case "PD_control"
          [proportional_gain, derivative_gain] = config.getPDControllerGain();
          controller = PDController(proportional_gain, derivative_gain);
        otherwise
          error("ERROR: Failed to set controller. Invalid controller type is specified.");
      end
    end

  end

end
% EOF