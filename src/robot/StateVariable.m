classdef StateVariable
  % State Variable
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    q     (:, 1) double  % Joint angle
    qd    (:, 1) double  % Joint angular velocity
    qdd   (:, 1) double  % Joint angular acceleration
    v0    (3, 1) double  % Base linear velocity
    w0    (3, 1) double  % Base angular velocity
    vd0   (3, 1) double  % Base linear acceleration
    wd0   (3, 1) double  % Base angular acceleration
    vv    (3, :) double  % Linear velocity of each link CoM
    ww    (3, :) double  % Angular velocity of each link CoM
    vd    (3, :) double  % Linear acceleration of each link CoM
    wd    (3, :) double  % Angular acceleration of each link CoM
    R0    (3, 1) double  % Base position
    Q0    (3, 1) double  % Base orientation (Euler)
    A0    (3, 3) double  % Base orientation (DCM)
    Qtn0  (4, 1) double  % Base orientation (Quaternion)
    dQtn0 (4, 1) double  % Division of base quaternion
    Fe    (3, :) double  % External force applied to end point
    Te    (3, :) double  % External torque applied to end point
    F0    (3, 1) double  % External force applied to base
    T0    (3, 1) double  % External torque applied to base
    tau   (:, 1) double  % Torque applied to each joint

    RR    (3, :) double  % Link positions
    AA    (3, :) double  % Link orientations (DCM)
  end
  properties (SetAccess = private, GetAccess = public)
    contact_state_ ContactState;
    is_supporting_ (1, :) logical;
    is_grasping_ (1, :) logical;
    is_slipping_ (1, :) logical;
  end

  %% Methods called only from Robot
  methods (Access = ?Robot)

    function SV = StateVariable(kNumJoints, kNumLimb)
    % StateVariable() Constructor
    %   Input - num_joints: Total number of joints
      arguments (Input)
        kNumJoints (1, 1) {mustBeA(kNumJoints, "uint8")};
        kNumLimb   (1, 1) {mustBeA(kNumLimb, "uint8")};
      end

      SV.q   = zeros(kNumJoints, 1);
      SV.qd  = zeros(kNumJoints, 1);
      SV.qdd = zeros(kNumJoints, 1);

      SV.v0  = zeros(3, 1);
      SV.w0  = zeros(3, 1);
      SV.vd0 = zeros(3, 1);
      SV.wd0 = zeros(3, 1);

      SV.vv = zeros(3, kNumJoints);
      SV.ww = zeros(3, kNumJoints);
      SV.vd = zeros(3, kNumJoints);
      SV.wd = zeros(3, kNumJoints);

      SV.R0 = zeros(3, 1);
      SV.Q0 = zeros(3, 1);
      SV.A0 = eye(3, 3);
      SV.Qtn0 = zeros(4, 1);
      SV.dQtn0 = zeros(4, 1);

      SV.Fe = zeros(3, kNumJoints);
      SV.Te = zeros(3, kNumJoints);
      SV.F0 = zeros(3, 1);
      SV.T0 = zeros(3, 1);

      SV.tau = zeros(kNumJoints, 1);

      SV.RR = zeros(3, kNumJoints);
      SV.AA = zeros(3, 3 * kNumJoints);

      SV.contact_state_ = ContactState(kNumLimb);
      SV.is_supporting_ = false(1, kNumLimb);
      SV.is_grasping_ = false(1, kNumLimb);
      SV.is_slipping_ = false(1, kNumLimb);
      % SV.contact_state_ = SV.contact_state_.detectEECollision(robot, terrain);
    end

    function SV = calcLinkPose(SV, LP)
    % calcLinkPose()
    %   Calculate positions and orientations (DCM) of all links
    %   Input - LP: Link parameters
      arguments (Input)
        SV;
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
      end
      SV_tmp = SV.clone();

      SV_tmp = calc_aa(LP, SV_tmp);
      SV.AA = SV_tmp.AA;

      SV_tmp = calc_pos(LP, SV_tmp);
      SV.RR = SV_tmp.RR;
    end

    function SV = overwrite(SV, state_variables)
    % overwrite()
    %   Overwrite all property values of state variables
    %   Input - state_variables: State variables with value for overwriting
      arguments (Input)
        SV;
        state_variables (1, 1) {mustBeA(state_variables, "struct")};
      end

      field_name = fieldnames(state_variables);
      prop_name = properties(SV);
      if (~all(strcmp(prop_name, field_name)))
        error("ERROR: Failed to overwrite state variables.");
      end

      for i = 1 : length(prop_name)
        SV.(prop_name{i, 1}) = state_variables.(field_name{i, 1});
      end
    end

    function SV = detectEECollision(SV, terrain, EE_positions, EE_orientations_dcm)
      SV.contact_state_ = SV.contact_state_.detectEECollision(terrain, ...
        EE_positions, EE_orientations_dcm, SV.is_grasping_);
    end

  end

  %% Public Methods
  methods (Access = public)

    function cloned_SV = clone(original_SV)
    % clone()
    %   Return state variables (struct) which have same values as properties of original state
    %   variables.
      prop_name = properties(original_SV);
      for i = 1 : length(prop_name)
        cloned_SV.(prop_name{i, 1}) = original_SV.(prop_name{i, 1});
      end
    end

  end

  %% Setter
  methods (Access = public)

    function SV = setJointAngularPositions(SV, joint_angles)
      arguments (Input)
        SV;
        joint_angles (:, 1) {mustBeA(joint_angles, "double")};
      end
      if (length(joint_angles) ~= length(SV.q))
        error("ERROR: Failed to set joint angles. " + "Number of joint angles is not correct.");
      end
      SV.q = joint_angles;
    end
    function SV = setJointTorque(SV, joint_torque)
      arguments (Input)
        SV;
        joint_torque (:, 1) {mustBeA(joint_torque, "double")};
      end
      if (length(joint_torque) ~= length(SV.tau))
        error("ERROR: Failed to set joint torque. " + "Number of joint torques is not correct.");
      end
      SV.tau = joint_torque;
    end

    function SV = setBasePosition(SV, base_position)
      arguments (Input)
        SV;
        base_position (3, 1) {mustBeA(base_position, "double")};
      end
      SV.R0 = base_position;
    end
    function SV = setBaseOrientationDCM(SV, base_orientation_dcm)
      arguments (Input)
        SV;
        base_orientation_dcm (3, 3) {mustBeA(base_orientation_dcm, "double")};
      end
      SV.A0 = base_orientation_dcm;
    end
    function SV = setBaseOrientationEuler(SV, base_orientation_euler)
      arguments (Input)
        SV;
        base_orientation_euler (3, 1) {mustBeA(base_orientation_euler, "double")};
      end
      SV.Q0 = base_orientation_euler;
    end

    function SV = applyExternalForces(SV, external_forces)
      arguments (Input)
        SV;
        external_forces (3, :) {mustBeA(external_forces, "double")};
      end
      if (size(external_forces, 2) ~= size(SV.Fe, 2))
        error("ERROR: Failed to apply external forces. " + ...
          "Number of external forces is not correct.");
      end
      SV.Fe = external_forces;
    end

    function SV = setContactPose(SV, contact_position, contact_orientation_dcm)
      SV.contact_state_ = SV.contact_state_.setContactPose(contact_position, contact_orientation_dcm);
    end
    function SV = setIsSupporting(SV, limb_ids_to_be_updated, next_state)
      for limb_id = 1 : size(SV.is_supporting_, 2)
        if (any(limb_id == limb_ids_to_be_updated))
          SV.is_supporting_(1, limb_id) = next_state;
        end
      end
    end
    function SV = setIsGrasping(SV, limb_id, next_state)
      SV.is_grasping_(1, limb_id) = next_state;
    end
    function SV = setIsSlipping(SV, limb_id, next_state)
      SV.is_slipping_(1, limb_id) = next_state;
    end

  end

  %% Getter
  methods (Access = public)
    function joint_angles = getJointAngularPosition(SV)
      joint_angles = SV.q;
    end
    function joint_angular_velocity = getJointAngularVelocity(SV)
      joint_angular_velocity = SV.qd;
    end
    function joint_angylar_acceleration = getJointAngularAcceleration(SV)
      joint_angylar_acceleration = SV.qdd;
    end

    function base_position = getBasePosition(SV)
      base_position = SV.R0;
    end
    function base_orientation_DCM = getBaseOrientationDCM(SV)
      base_orientation_DCM = SV.A0;
    end
    function base_orientation_euler = getBaseOrientationEuler(SV)
      base_orientation_euler = SV.Q0;
    end
    function base_linear_velocity = getBaseLinearVelocity(SV)
      base_linear_velocity = SV.v0;
    end
    function base_angular_velocity = getBaseAngularVelocity(SV)
      base_angular_velocity = SV.w0;
    end
    function base_linear_acceleration = getBaseLinearAcceleration(SV)
      base_linear_acceleration = SV.vd0;
    end
    function base_angular_acceleration = getBaseAngularAcceleration(SV)
      base_angular_acceleration = SV.wd0;
    end

    function ground_reaction_force = getGroundReactionForce(SV, LP)
      arguments (Input)
        SV;
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
      end
      [~, EE] = find(LP.getEndLink() == 1);
      ground_reaction_force = SV.Fe(:, EE);
    end
    function ground_reaction_moment = getGroundReactionMoment(SV, LP)
      arguments (Input)
        SV;
        LP (1, 1) {mustBeA(LP, "LinkParameters")};
      end
      [~, EE] = find(LP.getEndLink() == 1);
      ground_reaction_moment = SV.Te(:, EE);
    end

    function joint_torque = getJointTorque(SV)
      joint_torque = SV.tau;
    end

    function is_supporting = getIsSupporting(SV)
      is_supporting = SV.is_supporting_;
    end
    function is_grasping = getIsGrasping(SV)
      is_grasping = SV.is_grasping_;
    end
  end

end
% EOF
