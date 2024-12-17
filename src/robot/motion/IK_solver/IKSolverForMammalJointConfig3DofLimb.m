classdef IKSolverForMammalJointConfig3DofLimb < handle
% Position inverse kinematics solver for a 3 DOF manipulator that has a mammal joint configuration
%
% Created     : 2021.01.18 by Kentaro Uno
% Last updated: 2021.02.12 by Kentaro Uno

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kLimbId_ (1, 1) uint8;

    kLegConfigType_ (1, 1) string;

    % Rotational relationship of links frames to define the offset angle
    theta_1_ (1, 1) double;
    theta_2_ (1, 1) double;

    pos_vec_01_ (3, 1) double;  % Base  (link 0) to Hip   (link 1) position vector
    pos_vec_12_ (3, 1) double;  % Hip   (link 1) to Thigh (link 2) position vector
    pos_vec_23_ (3, 1) double;  % Thigh (link 2) to Shank (link 3) position vector
    pos_vec_3e_ (3, 1) double;  % Shank (link 3) to end-effector (link "e") position vector
  end

  %% Public Methods
  methods (Access = public)

    function IK_solver = IKSolverForMammalJointConfig3DofLimb(LP, limb_id)
    % IKSolverForMammalJointConfig3DofLimb() Constructor
      arguments (Input)
        LP      (1, 1) {mustBeA(LP,      "LinkParameters")};
        limb_id (1, 1) {mustBeA(limb_id, "uint8")};
      end

      IK_solver.kLimbId_ = limb_id;

      IK_solver.kLegConfigType_ = LP.getMammalLegConfigType();

      [IK_solver.theta_1_, IK_solver.theta_2_] = LP.getMammalConfigOffsetAngles();

      IK_solver.setLinkPositionVectors(LP, limb_id);
    end

    function joint_angle = solve(IK_solver, base_position, base_orientation, EE_position)
      arguments (Input)
        IK_solver;
        base_position (3, 1) {mustBeA(base_position, "double")};
        base_orientation (3, 3) {mustBeA(base_orientation, "double")};
        EE_position (3, 1) {mustBeA(EE_position, "double")};
      end

      % Base (link 0) to end-effector (link "e") position vector, i.e. input of IK
      p0e_in_Base_frame = base_orientation' * (EE_position - base_position);

      joint_angle_tmp(1, 1) = IK_solver.solveB2HJointAngle(p0e_in_Base_frame);

      [joint_angle_tmp(2, 1), A, B] = IK_solver.solveH2TJointAngle(p0e_in_Base_frame, joint_angle_tmp(1, 1));

      joint_angle_tmp(3, 1) = IK_solver.solveT2SJointAngle(joint_angle_tmp(2, 1), A, B);

      % Adjust the frame for SpaceDyn from frame for IK
      joint_angle(1, 1) =  joint_angle_tmp(1, 1);
      joint_angle(2, 1) = -joint_angle_tmp(2, 1);
      joint_angle(3, 1) = -joint_angle_tmp(3, 1);

      % Adjust the solutions to be described radians from -pi to pi
      if (joint_angle(1, 1) > pi())
        joint_angle(1, 1) = joint_angle(1, 1) - 2.0 * pi();
      elseif (joint_angle(1, 1) < -pi())
        joint_angle(1, 1) = joint_angle(1, 1) + 2.0 * pi();
      end

      if (joint_angle(2, 1) > pi())
        joint_angle(2, 1) = joint_angle(2, 1) - 2.0 * pi();
      elseif (joint_angle(2, 1) < pi())
        joint_angle(2, 1) = joint_angle(2, 1) + 2.0 * pi();
      end

      if (joint_angle(3, 1) > pi())
        joint_angle(3, 1) = joint_angle(3, 1) - 2.0 * pi();
      elseif (joint_angle(3, 1) < pi())
        joint_angle(3, 1) = joint_angle(3, 1) + 2.0 * pi();
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function setLinkPositionVectors(IK_solver, LP, limb_id)
      arguments (Input)
        IK_solver;
        LP      (1, 1) {mustBeA(LP, "LinkParameters")};
        limb_id (1, 1) {mustBeA(limb_id, "uint8")};
      end

      c0 = LP.getPositionVectorFromBaseCoMToJoint();
      cc = LP.getPositionVectorFromLinkCoMToJoint();
      ce = LP.getPositionVectorFromEndLinkCoMToEndPoint();
      kJoints = LP.getJoints();

      p01 = c0(:, kJoints(1, limb_id));

      p12_tmp = cc(:, kJoints(1, limb_id), kJoints(1, limb_id) + 1) - cc(:, kJoints(1, limb_id)  , kJoints(1, limb_id));
      p23_tmp = cc(:, kJoints(1, limb_id) + 1, kJoints(1, limb_id) + 2) - cc(:, kJoints(1, limb_id) + 1, kJoints(1, limb_id) + 1);
      p3e_tmp = ce(:, kJoints(1, limb_id) + 2) - cc(:, kJoints(1, limb_id) + 2, kJoints(1, limb_id) + 2);

      % Adjusting the frame for IK from frame of SpaceDyn
      p12(1, 1) =  p12_tmp(3, 1);
      p12(2, 1) =  p12_tmp(2, 1);
      p12(3, 1) = -p12_tmp(1, 1);
      p23(1, 1) =  p23_tmp(2, 1);
      p23(2, 1) = -p23_tmp(3, 1);
      p23(3, 1) = -p23_tmp(1, 1);
      p3e(1, 1) =  p3e_tmp(2, 1);
      p3e(2, 1) = -p3e_tmp(3, 1);
      p3e(3, 1) = -p3e_tmp(1, 1);

      IK_solver.pos_vec_01_ = p01;
      IK_solver.pos_vec_12_ = p12;
      IK_solver.pos_vec_23_ = p23;
      IK_solver.pos_vec_3e_ = p3e;
    end

    function q1 = solveB2HJointAngle(IK_solver, p0e)
    % solveB2HJointAngle()
    %   Calculate joint angle of base to hip joint
    % Input - p0e: position vector from base to end-effector in base frame
      theta_1 = IK_solver.theta_1_;
      theta_2 = IK_solver.theta_2_;
      p01 = IK_solver.pos_vec_01_;
      p12 = IK_solver.pos_vec_12_;
      p23 = IK_solver.pos_vec_23_;
      p3e = IK_solver.pos_vec_3e_;

      % a1, b1, c1: temporary variable to calculate B2H joint angle
      a1 =  cos(theta_1) * sin(theta_2) * (p0e(1, 1) - p01(1, 1)) + ...
            sin(theta_1) * sin(theta_2) * (p0e(2, 1) - p01(2, 1)) + ...
            cos(theta_2) * (p0e(3, 1) - p01(3, 1));

      b1 = -sin(theta_1) * (p0e(1, 1) - p01(1, 1)) + cos(theta_1) * (p0e(2, 1) - p01(2, 1));

      c1 = p12(2, 1) + p23(2, 1) + p3e(2, 1);

      q1 = atan2(a1, b1) + atan2(sqrt(a1 ^ 2 + b1 ^ 2 - c1 ^ 2), c1);
    end

    function [q2, A, B] = solveH2TJointAngle(IK_solver, p0e, q1)
    % solveH2TJointAngle()
    %   Calculate joint angle of hip to thigh joint
    % Input - p0e: position vector from base to end-effector in base frame
    %       - q1: joint angle of base to hip joint
      theta_1 = IK_solver.theta_1_;
      theta_2 = IK_solver.theta_2_;
      p01 = IK_solver.pos_vec_01_;
      p12 = IK_solver.pos_vec_12_;
      p23 = IK_solver.pos_vec_23_;
      p3e = IK_solver.pos_vec_3e_;

      % A, B, a2, b2, c2: temporary variable to calculate H2T joint angle
      A = cos(theta_1) * cos(theta_2) * ( p0e(1, 1) - p01(1, 1) ) + ...
          sin(theta_1) * cos(theta_2) * ( p0e(2, 1) - p01(2, 1) ) - ...
          sin(theta_2) * (p0e(3, 1) - p01(3, 1)) - ...
          p12(1, 1);

      B = ( sin(theta_1) * sin(q1) + cos(theta_1) * sin(theta_2) * cos(q1)) * (p0e(1, 1) - p01(1, 1)) + ...
          (-cos(theta_1) * sin(q1) + sin(theta_1) * sin(theta_2) * cos(q1)) * (p0e(2, 1) - p01(2, 1)) + ...
            cos(theta_2) * cos(q1) * (p0e(3, 1) - p01(3, 1)) - ...
            p12(3, 1);

      a2 = 2 * (A * p23(3, 1) - B * p23(1, 1));

      b2 = 2 * (A * p23(1, 1) + B * p23(3, 1));

      c2 = A ^ 2 + B ^ 2 + p23(1, 1) ^ 2 + p23(3, 1) ^ 2 - p3e(1, 1) ^ 2 - p3e(3, 1) ^ 2;

      % If we want xx config solution, the sign of the hip to thigh joint solution should be flipped between front legs and hind legs.
      kLimbId = IK_solver.kLimbId_;
      switch (IK_solver.kLegConfigType_)
        case "oo"
          if (kLimbId == 1 || kLimbId == 4)
            q2 = atan2(a2, b2) - atan2(sqrt(a2 ^ 2 + b2 ^ 2 - c2 ^ 2), c2);
          elseif (kLimbId == 2 || kLimbId == 3)
            q2 = atan2(a2, b2) + atan2(sqrt(a2 ^ 2 + b2 ^ 2 - c2 ^ 2), c2);
          end
        case "xx"
          if (kLimbId == 1 || kLimbId == 4)
            q2 = atan2(a2, b2) + atan2(sqrt(a2 ^ 2 + b2 ^ 2 - c2 ^ 2), c2);
          elseif (kLimbId == 2 || kLimbId == 3)
            q2 = atan2(a2, b2) - atan2(sqrt(a2 ^ 2 + b2 ^ 2 - c2 ^ 2), c2);
          end
        case "M"
          if (kLimbId == 1 || kLimbId == 4)
            q2 = atan2(a2, b2) + atan2(sqrt(a2 ^ 2 + b2 ^ 2 - c2 ^ 2), c2);
          elseif (kLimbId == 2 || kLimbId == 3)
            q2 = atan2(a2, b2) + atan2(sqrt(a2 ^ 2 + b2 ^ 2 - c2 ^ 2), c2);
          end
        otherwise
          error("ERROR: Failed to calculate H2T joint angle.");
      end
    end

    function q3 = solveT2SJointAngle(IK_solver, q2, A, B)
    % solveT2SJointAngle()
    %   Calculate joint angle of thigh to shank joint
    % Input - q2: joint angle of hip to thigh joint
    %       - A, B: Temporary variable to calculate joint angle
      p23 = IK_solver.pos_vec_23_;
      p3e = IK_solver.pos_vec_3e_;

      q3 =  2 * pi() + ...
            atan2(A * cos(q2) - B * sin(q2) - p23(1, 1), A * sin(q2) + B * cos(q2) - p23(3, 1)) - ...
            atan2(p3e(1, 1), p3e(3, 1));
    end

  end

end  % IKSolverForMammalJointConfig3DofLimb
