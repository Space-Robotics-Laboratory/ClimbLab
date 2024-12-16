classdef JointController < handle
% JointController
% Joint controller
%
% Created     : 2024.05.20 by Masazumi Imai
% Last updated: 2024.12.13 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kType_ (1, 1) string;
  end
  properties (SetAccess = public, GetAccess = public)
    controller_;
  end

  %% Public Methods
  methods (Access = public)

    function joint_controller = JointController(config_joint_controller)
      arguments (Input)
        config_joint_controller (1, 1) {mustBeA(config_joint_controller, "ConfigJointController")};
      end

      joint_controller.kType_ = config_joint_controller.getType();
      joint_controller.setController(config_joint_controller);
    end

    function robot = control(joint_controller, robot)
      arguments (Input)
        joint_controller;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      joint_torque = joint_controller.controller_.calcJointTorque(robot);

      robot.setJointTorque(joint_torque);
    end

  end

  %% Private Methods
  methods (Access = private)

    function setController(joint_controller, config_joint_controller)
      switch (joint_controller.kType_)
        case "PD_control"
          controller = PDController(config_joint_controller);
        otherwise
          error("ERROR: Failed to set controller. Invalid controller type is specified.");
      end

      joint_controller.controller_ = controller;
    end

  end

end  % JointController
