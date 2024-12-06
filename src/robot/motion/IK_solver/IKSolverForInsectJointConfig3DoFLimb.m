classdef IKSolverForInsectJointConfig3DoFLimb
% Position inverse kinematics solver for a 3 DOF manipulator that has a insect joint configuration
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    yaw_Base2Coxa_in_Base_frame (1, 1) double  % yaw rotation of Base_to_Coxa joint seen from Base frame
    pos_vec_01 (3, 1) double;  % Base (link 0) to Coxa (link 1) position vector
    pos_vec_12 (3, 1) double;  % Coxa (link 1) to Femur (link 2) position vector
    pos_vec_23 (3, 1) double;  % Femur (link 2) to Tibia (link 3)position vector
    pos_vec_3e (3, 1) double;  % Tibia (link 3) to endeffector (link "e") position vector
  end

  %% Public Methods
  methods (Access = public)

    % Constructor
    function IK_solver = IKSolverForInsectJointConfig3DoFLimb()
      IK_solver.yaw_Base2Coxa_in_Base_frame = 0.0;
      IK_solver.pos_vec_01 = zeros(3, 1);
      IK_solver.pos_vec_12 = zeros(3, 1);
      IK_solver.pos_vec_23 = zeros(3, 1);
      IK_solver.pos_vec_3e = zeros(3, 1);
    end

    function IK_solver = setLinksPositionVectors(IK_solver, yaw_base2coxa, position_vectors)
      arguments (Input)
        IK_solver;
        yaw_base2coxa (:, 1) {mustBeA(yaw_base2coxa, "double")};
        position_vectors (3, :) {mustBeA(position_vectors, "double")};
      end
      IK_solver.yaw_Base2Coxa_in_Base_frame = yaw_base2coxa;
      IK_solver.pos_vec_01 = position_vectors(:, 1);
      IK_solver.pos_vec_12 = position_vectors(:, 2);
      IK_solver.pos_vec_23 = position_vectors(:, 3);
      IK_solver.pos_vec_3e = position_vectors(:, 4);
    end

    function joint_angle = solve(IK_solver, base_position, base_orientation, EE_position)
      arguments (Input)
        IK_solver;
        base_position (3, 1) {mustBeA(base_position, "double")};
        base_orientation (3, 3) {mustBeA(base_orientation, "double")};
        EE_position (3, 1) {mustBeA(EE_position, "double")};
      end
      % Base (link 0) to endeffector (link "e") position vector, i.e. input of IK
      p0e_in_Inertia_frame = EE_position - base_position;
      p0e_in_Base_frame = base_orientation' * p0e_in_Inertia_frame;

      joint_angle_tmp(1, 1) = IK_solver.solveB2CJointAngle(p0e_in_Base_frame);
      [joint_angle_tmp(2, 1), A, B] = IK_solver.solveC2FJointAngle(p0e_in_Base_frame, ...
        joint_angle_tmp(1, 1));
      joint_angle_tmp(3, 1) = IK_solver.solveF2TJointAngle(joint_angle_tmp(2, 1), A, B);

      % Adjust the frame for SpaceDyn from frame for IK
      joint_angle(1, 1) = joint_angle_tmp(1, 1);
      joint_angle(2, 1) = - joint_angle_tmp(2, 1);
      joint_angle(3, 1) = - (joint_angle_tmp(3, 1) + pi / 2);

      % Adjust the solutions to be described radians from -pi to pi
      if joint_angle(1, 1) > pi
        joint_angle(1, 1) = joint_angle(1, 1) - 2*pi;
      elseif joint_angle(1, 1) < -pi
        joint_angle(1, 1) = joint_angle(1, 1) + 2*pi;
      end

      if joint_angle(2, 1) > pi
        joint_angle(2, 1) = joint_angle(2, 1) - 2*pi;
      elseif joint_angle(2, 1) < -pi
        joint_angle(2, 1) = joint_angle(2, 1) + 2*pi;
      end

      if joint_angle(3, 1) > pi
        joint_angle(3, 1) = joint_angle(3, 1) - 2*pi;
      elseif joint_angle(3, 1) < -pi
        joint_angle(3, 1) = joint_angle(3, 1) + 2*pi;
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function q1 = solveB2CJointAngle(IK_solver, p0e)
      alpha = IK_solver.yaw_Base2Coxa_in_Base_frame;
      p01 = IK_solver.pos_vec_01;
      p12 = IK_solver.pos_vec_12;
      p23 = IK_solver.pos_vec_23;
      p3e = IK_solver.pos_vec_3e;

      % a1, b1, c1: temporary valuabled to calculate B2C joint angle
      a1 = - p0e(1, 1) + p01(1, 1);
      b1 =   p0e(2, 1) - p01(2, 1);
      c1 =   p12(2, 1) + p23(2, 1) + p3e(2, 1);

      % Singurarity check
      if ~isreal(sqrt(a1^2 + b1^2 - c1^2))
        return;
      end

      q1 = atan2( a1 , b1 ) + atan2( sqrt(a1^2 + b1^2 - c1^2), c1 ) - alpha;
    end

    function [q2, A, B] = solveC2FJointAngle(IK_solver, p0e, q1)
      alpha = IK_solver.yaw_Base2Coxa_in_Base_frame;
      p01 = IK_solver.pos_vec_01;
      p12 = IK_solver.pos_vec_12;
      p23 = IK_solver.pos_vec_23;
      p3e = IK_solver.pos_vec_3e;

      % A, B, a2, b2, c2: temporary valuabled to calculate C2F joint angle
      A = (p0e(1, 1) - p01(1, 1)) * cos(alpha + q1) + ...
          (p0e(2, 1) - p01(2, 1)) * sin(alpha + q1) - p12(1, 1);
      B = p0e(3, 1) - p01(3, 1) - p12(3, 1);
      a2 = 2 * (B * p23(1, 1) + A * p23(3, 1));
      b2 = 2 * (A * p23(1, 1) + B * p23(3, 1));
      c2 = A^2 + B^2 + p23(1, 1)^2 + p23(3, 1)^2 - p3e(1, 1)^2 - p3e(3, 1)^2;

      % Singurarity check
      if ~isreal(sqrt(a2^2 + b2^2 - c2^2))
        return;
      end

      q2 = - ( atan2( a2 , b2 ) + atan2( sqrt(a2^2 + b2^2 - c2^2), c2 ) );
    end

    function q3 = solveF2TJointAngle(IK_solver, q2, A, B)
      p23 = IK_solver.pos_vec_23;
      p3e = IK_solver.pos_vec_3e;

      q3 = 2 * pi + ...
        atan2( A * cos(q2) - B * sin(q2) - p23(1, 1), A * sin(q2) + B * cos(q2) - p23(3, 1) ) - ...
        atan2( p3e(1, 1), p3e(3, 1) );
    end

  end

end
% EOF