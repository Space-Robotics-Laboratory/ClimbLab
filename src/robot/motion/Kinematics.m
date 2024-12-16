classdef Kinematics
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

      kJointConfig = robot.LP_.getJointAllocationType();
      kNumLimb = robot.LP_.getNumberOfLimb();
      kNumJointsPerLimb = robot.LP_.getNumberOfJointsPerLimb();

      switch (kJointConfig)
        % case "mammal"
        case "insect"
          for limb_id = 1 : kNumLimb
            num_joints = kNumJointsPerLimb(1, limb_id);
            if (num_joints == 3)
              solver(limb_id, 1) = IKSolverForInsectJointConfig3DoFLimb();
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

      [yaw_base_to_limb_root, position_vectors_of_links] = ...
        kinematics.calcLinksPositionVectors(robot);
      for limb_id = 1 : kNumLimb
        kinematics.IK_solver_(limb_id, 1) = ...
          kinematics.IK_solver_(limb_id, 1).setLinksPositionVectors( ...
          yaw_base_to_limb_root(limb_id, 1), position_vectors_of_links(:, :, limb_id));
      end
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

      num_limb = length(kinematics.IK_solver_);
      joints = LP.getJoints();
      EE_position = zeros(3, num_limb);
      EE_orientation_dcm = zeros(3, 3 * num_limb);

      for limb_id = 1 : num_limb
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

      num_limb = length(kinematics.IK_solver_);
      joint_angles = double.empty;

      for limb_id = 1 : num_limb
        joint_angle_for_each_limb = kinematics.IK_solver_(limb_id, 1).solve( ...
          base_position, base_orientation_dcm, EE_position(:, limb_id));
        joint_angles = vertcat(joint_angles, joint_angle_for_each_limb);
      end
    end

    % function generalized_jacobian = computeGeneralizedJacobianForEndEffector(kinematics)
    % end
    % function jacobian = computeJointToLinkJacobian(kinematics)
    % end
    % function jacobian_derivative = computeJointToLinkJacobianDerivative(kinematics)
    % end
    % function jacobian = computeBaseToLinkJacobian(kinematics)
    % end
    % function jacobian_derivative = computeBaseToLinkJacobianDerivative(kinematics)
    % end

  end

  %% Private Methods
  methods (Access = private)

    function [yaw_base_to_limb_root, position_vectors_of_links] = calcLinksPositionVectors(~, robot)
      robot_type = robot.getType();
      Qi = robot.LP_.getRotationalRelationshipOfLinkFrames();
      joints = robot.LP_.getJoints();
      c0 = robot.LP_.getPositionVectorFromBaseCoMToJoint();
      cc = robot.LP_.getPositionVectorFromLinkCoMToJoint();
      ce = robot.LP_.getPositionVectorFromEndLinkCoMToEndPoint();
      num_limb = robot.LP_.getNumberOfLimb();

      yaw_base_to_limb_root = zeros(num_limb, 1);
      if (startsWith(robot_type, "HubRobo"))
        for limb_id = 1 : num_limb
          yaw_base_to_limb_root(limb_id, 1) = Qi(3, joints(1, limb_id));
          % Base (link 0) to Coxa (link 1) position vector
          p01 = c0(:, joints(1, limb_id));
          % Coxa (link 1) to Femur (link 2) position vector
          p12 = cc(:, joints(1, limb_id), joints(1, limb_id) + 1) - ...
            cc(:, joints(1, limb_id), joints(1, limb_id));
          % Femur (link 2) to Tibia (link 3)position vector
          p23_tmp = cc(:, joints(1, limb_id)+1, joints(1, limb_id) + 2) - ...
            cc(:, joints(1, limb_id)+1, joints(1, limb_id) + 1);
          % adjusting the frame for IK from frame of SpaceDyn
          p23(1, 1) = p23_tmp(1, 1); p23(2, 1) = - p23_tmp(3, 1); p23(3, 1) = p23_tmp(2, 1);
          % Tibia (link 3) to end-effector (link "e") position vector
          p3e_tmp = ce(:, joints(1, limb_id) + 2) - ...
            cc(:, joints(1, limb_id)+2, joints(1, limb_id) + 2);
          % adjusting the frame for IK from frame of SpaceDyn
          p3e(1, 1) = p3e_tmp(2, 1); p3e(2, 1) = - p3e_tmp(3, 1); p3e(3, 1) = - p3e_tmp(1, 1);

          position_vectors_of_links(:, :, limb_id) = [p01, p12, p23, p3e];
        end
      end
    end

  end

end  % Kinematics
