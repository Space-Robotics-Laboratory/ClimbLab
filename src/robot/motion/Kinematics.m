classdef Kinematics < handle
  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    IK_solver_ (:, 1);
  end

  %% Public Methods
  methods (Access = public)

    function kinematics = Kinematics(robot)
    % Kinematics() Constructor
      arguments (Input)
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      LP = robot.getLinkParameter();
      kJointConfig = robot.LP_.getJointAllocationType();
      kNumLimb = robot.LP_.getNumberOfLimb();
      kNumJointsPerLimb = robot.LP_.getNumberOfJointsPerLimb();

      switch (kJointConfig)
        % case "mammal"
        case "insect"
          for limb_id = 1 : kNumLimb
            num_joints = kNumJointsPerLimb(1, limb_id);
            if (num_joints == 3)
              solver(limb_id, 1) = IKSolverForInsectJointConfig3DofLimb(LP, limb_id);
            % elseif (num_joints == 4)
            % elseif (num_joints == 5)
            else
              error("No IK solver!!");
            end
          end
        otherwise
            error("Invalid joint allocation type is specified." + newline + ...
              "Check ""joint_allocation_type"" defined in LP file.");
      end
      kinematics.IK_solver_ = solver;
    end

    function [EE_position, EE_orientation_dcm] = computeForward(kinematics, LP, SV)
    % computeForward()
    %   Compute forward kinematics using SpaceDyn function "f_kin_e()"
    %   Input - LP: Link parameters
    %           SV: State variables
    %   Output - EE_position: End-Effector positions of each limb
    %            EE_orientation_dcm: End-Effector orientations (DCM) of each limb
      arguments (Input)
        kinematics;
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
        SV (1, 1) {mustBeA(SV, "StateVariable")};
      end

      kNumLimb = length(kinematics.IK_solver_);
      joints = LP.getJoints();
      EE_position = zeros(3, kNumLimb);
      EE_orientation_dcm = zeros(3, 3 * kNumLimb);

      for limb_id = 1 : kNumLimb
        [EE_position(:, limb_id), EE_orientation_dcm(:, 3*limb_id-2:3*limb_id)] = ...
          f_kin_e(LP, SV, joints(:, limb_id));
      end
    end

    function joint_angles = computeInverse(kinematics, ...
        base_position, base_orientation_dcm, EE_position)
    % computeInverse()
    %   Compute inverse kinematics
    %   Input - base_position: Position of the robot base
    %         - base_orientation_dcm: Orientation (DCM) of the robot base
    %         - EE_position: Positions of each End-Effector
    %   Output - joint_angles: Joint angular positions of all joints
      arguments (Input)
        kinematics;
        base_position (3, 1) {mustBeA(base_position, "double")};
        base_orientation_dcm (3, 3) {mustBeA(base_orientation_dcm, "double")};
        EE_position (3, :) {mustBeA(EE_position, "double")};
      end

      kNumLimb = length(kinematics.IK_solver_);
      joint_angles = double.empty;

      for limb_id = 1 : kNumLimb
        joint_angle_for_each_limb = kinematics.IK_solver_(limb_id, 1).solve( ...
          base_position, base_orientation_dcm, EE_position(:, limb_id));
        joint_angles = vertcat(joint_angles, joint_angle_for_each_limb);
      end
    end

  end

end  % Kinematics
