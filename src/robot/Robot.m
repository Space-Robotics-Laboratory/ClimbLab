classdef Robot < handle

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kType_ (1, 1) string;
    LP_ LinkParameters;
    SV_ StateVariable;
    des_SV_ StateVariable;

    kinematics_ Kinematics;
    dynamics_ Dynamics;

    EE_position_ (3, :) double;
    EE_orientation_dcm_ (3, :) double;
    % EE_orientation_euler_rad (3, :) double;
    % EE_orientation_euler_deg (3, :) double;

    base_height_in_Surface_ (1, 1) double;

    gripper_detachment_detection_method_ (1, 1) string;

    graphics_ RobotGraphics;
  end

  %% Public methods
  methods (Access = public)

    function robot = Robot(config_robot, world, terrain)
    % Robot() Constructor
      arguments (Input)
        config_robot (1, 1) {mustBeA(config_robot, "ConfigRobot")};
        world (1, 1) {mustBeA(world, "World")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end
      robot.kType_ = config_robot.getRobotType();
      robot.LP_ = LinkParameters(robot.kType_ + "_LP");

      kNumLimb = robot.LP_.getNumberOfLimb();
      robot.SV_ = StateVariable(robot.LP_.getNumberOfJoints(), kNumLimb);
      robot.initializeBasePose(config_robot, terrain);
      robot.des_SV_ = StateVariable(robot.LP_.getNumberOfJoints(), kNumLimb);

      robot.EE_position_ = zeros(3, kNumLimb);
      robot.initializeEEPosition(config_robot, terrain);
      robot.EE_orientation_dcm_ = zeros(3, 3 * kNumLimb);
      % robot.EE_orientation_euler_rad = zeros(3, kNumLimb);
      % robot.EE_orientation_euler_deg = zeros(3, kNumLimb);

      robot.kinematics_ = Kinematics(robot);
      base_position = robot.SV_.getBasePosition();
      base_orientation_dcm = robot.SV_.getBaseOrientationDCM();
      joint_angles = robot.kinematics_.computeInverse(base_position, base_orientation_dcm, ...
        robot.getEEPosition());
      robot.SV_.setJointAngularPositions(joint_angles);
      robot.SV_.calcLinkPose(robot.LP_);
      robot.forwardKinematics();

      robot.dynamics_ = Dynamics(world.getUseDynamics());

      robot.des_SV_.overwrite(robot.SV_.clone());

      robot.gripper_detachment_detection_method_ = config_robot.getGripperDetachmentDetectionMethod();
      robot.detectCollision(terrain);
      robot.des_SV_.setIsSupporting(1 : kNumLimb, true);
      robot.updateGripperState(terrain);

      if (~config_robot.getVisualizeRobot())
        return;
      end
      robot.graphics_ = RobotGraphics(config_robot, robot.LP_, robot.SV_);
    end

    function forwardKinematics(robot)
    % forwardKinematics()
    %   Compute forward kinematics
      [robot.EE_position_, robot.EE_orientation_dcm_] = robot.kinematics_.computeForward( ...
        robot.LP_, robot.SV_);
    end

    function robot = forwardDynamics(robot)
    % forwardDynamics()
    %   Compute forward dynamics
      robot = robot.dynamics_.computeForward(robot);
    end

    function detectCollision(robot, terrain)
    % detectCollision()
    %   Detect a new contact (or losing an old one) between the robot end-effector and the ground
    %   surface, and save contact pose of End-Effectors.
      arguments (Input)
        robot;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end
      robot.des_SV_.detectEECollision(terrain, robot.EE_position_, robot.EE_orientation_dcm_);
      robot.SV_.detectEECollision(terrain, robot.EE_position_, robot.EE_orientation_dcm_);
    end

    function calcGroundReactionForces(robot, terrain)
    % calcGroundReactionForces()
    %   Calculate external forces and moments on each limb based on a spring-damper model for the
    %   contact, which is based on the first contact point between the robot end-effector and the
    %   ground surface.
    %     F_e = -K * (x_e - x_c) - D * dx_e/dt
    %       F_e : Ground reaction force
    %       K   : Ground stiffness coefficient
    %       D   : Ground damping coefficient
    %       x_e : Current end-effector position or orientation
    %       x_c : Contact position or orientation
      arguments (Input)
        robot;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end

      kNumLimb = robot.LP_.getNumberOfLimb();
      kNumJoints = robot.LP_.getNumberOfJoints();
      [~, EndEffectors] = find(robot.LP_.getEndLink() == 1);
      EE_in_contact = robot.SV_.contact_state_.getInContact();
      contact_position = robot.SV_.contact_state_.getPosition();
      EE_is_grasping = robot.SV_.getIsGrasping();
      [Kf, Df, Km, Dm] = terrain.getGroundCoefficients();

      GJ = zeros(6, kNumJoints, kNumLimb);
      GRF = zeros(3, robot.LP_.getNumberOfJoints());

      for limb_id = 1 : kNumLimb
        EE = EndEffectors(1, limb_id);

        if (EE_is_grasping(1, limb_id) || EE_in_contact(1, limb_id))
          GJ(:, :, limb_id) = calc_gj(robot.LP_.clone(), robot.SV_.clone(), limb_id);
          EE_velocity(:, limb_id) = GJ(:, :, limb_id) * robot.SV_.getJointAngularVelocity();

          GRF(:, EE) = - Kf * (robot.EE_position_(:, limb_id) - contact_position(:, limb_id)) - ...
            Df * EE_velocity(1:3, limb_id);
        else
          GRF(:, EE) = zeros(3, 1);
        end
      end
      robot.SV_.applyExternalForces(GRF);
    end

    function updateDesiredGripperState(robot, ...
        time, foothold_planning, gait_planning, terrain)
      arguments (Input)
        robot;
        time              (1, 1) {mustBeA(time, "double")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
        gait_planning     (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
        terrain       (1, 1) {mustBeA(terrain, "Terrain")};
      end
      global d_time;
      persistent EE_position_last;

      kNumLimb = robot.LP_.getNumberOfLimb();
      swing_limb_id = foothold_planning.planner_.output_.getSwingLimbId();

      for limb_id = 1 : kNumLimb
        if (all(limb_id ~= swing_limb_id))
          continue;
        end

        swing_timings = gait_planning.scheduler_.output_.getSwingTimings();
        landing_timings = gait_planning.scheduler_.output_.getLandingTimings();

        % Release gripper of swing limb at swing motion start time
        if (abs(time - swing_timings(1, limb_id)) < eps)
          robot.des_SV_.setIsSupporting(limb_id, false);
        end

        desired_support_limb_id = robot.des_SV_.getIsSupporting();

        % Determining whether to close the gripper
        if (time < (swing_timings(1, limb_id) + landing_timings(1, limb_id)) / 2 ...
            || desired_support_limb_id(1, limb_id) ~= false)
          continue;
        end

        swing_EE_position = robot.EE_position_(:, limb_id);
        graspable_points = terrain.getGraspablePoints();
        near_GP = graspable_points.getNearestPoint(swing_EE_position);

        % Distance between swing limb End-Effector and nearest graspable point positions
        dist_EE_nearGP = norm(swing_EE_position - near_GP);

        distance_threshold = 0.001;  % TODO: should be set in config
        if (dist_EE_nearGP > distance_threshold)
          continue;
        end

        EE_linear_velocity = (swing_EE_position - EE_position_last(:, limb_id)) / d_time;
        norm_EE_linear_velocity = norm(EE_linear_velocity);

        velocity_threshold = 0.01;  % TODO: should be set in config
        if (norm_EE_linear_velocity <= velocity_threshold)
          robot.des_SV_.setIsSupporting(limb_id, true);
        end
      end

      % Store last end-effector positions
      EE_position_last = robot.EE_position_;
    end

    function updateGripperState(robot, terrain)
    % updateGripperState()
    %   1 - Detect a new contact (or losing an old one) between the end-effector and the ground
    %       surface.
    %   2 - Update the actual state of the gripper, if it is open or closed, based on the desired
    %       state and detachment conditions.
    %   3 - Obtain the contact position and orientation at first contact for further reaction forces
    %       calculation if the end-effector collides with the ground surface.
      arguments (Input)
        robot;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end

      % Check gripper detachment based on detection method
      switch (robot.gripper_detachment_detection_method_)
        case "none"
        case "max_holding_force"
          kNumLimb = robot.LP_.getNumberOfLimb();
          ground_reaction_force = robot.SV_.getGroundReactionForce(robot.LP_);
          F_grip = robot.LP_.getMaxEndurableGrippingForce();

          robot.detectCollision(terrain);
          EE_in_contact = robot.SV_.contact_state_.getInContact();
          contact_EE_position = robot.SV_.contact_state_.getPosition();
          contact_EE_orientation_dcm = robot.SV_.contact_state_.getOrientationDCM();

          for limb_id = 1 : kNumLimb
            % Swing limb EE is not grasping and does not cause slip
            desired_support_limb_id = robot.des_SV_.getIsSupporting();
            if (~desired_support_limb_id(1, limb_id))
              robot.SV_.setIsSupporting(limb_id, false);
              robot.SV_.setIsGrasping(limb_id, false);
              robot.SV_.setIsSlipping(limb_id, false);
              if (~EE_in_contact(1, limb_id))
                contact_EE_position(:, limb_id) = NaN(3, 1);
                contact_EE_orientation_dcm(:, 3*limb_id-2 : 3*limb_id) = NaN;
              end
              continue;
            end
            % Gripper detachment is NOT caused
            if (norm(ground_reaction_force(:, limb_id)) <= F_grip || EE_in_contact(1, limb_id))
              EE_is_supporting = robot.SV_.getIsSupporting();
              if (~EE_is_supporting(1, limb_id))
                robot.SV_.setIsSupporting(limb_id, true);
                contact_EE_position(:, limb_id) = ...
                  terrain.getNearestPointInWorldFrame(robot.EE_position_(:, limb_id));
                contact_EE_orientation_dcm(:, 3*limb_id-2 : 3*limb_id) = ...
                  robot.EE_orientation_dcm_(:, 3*limb_id-2 : 3*limb_id);
              end
              robot.SV_.setIsGrasping(limb_id, true);
              robot.SV_.setIsSlipping(limb_id, false);
            % Gripper detachment is caused when the Ground Reaction Force acting on End-Effector in
            % the pulling direction exceeded the maximum tolerable grasping force
            else
              robot.SV_.setIsSupporting(limb_id, false);
              robot.SV_.setIsGrasping(limb_id, false);
              robot.SV_.setIsSlipping(limb_id, true);
            end
          end
          robot.SV_.contact_state_.setContactPose(contact_EE_position, contact_EE_orientation_dcm);
        otherwise
          error("ERROR: Invalid gripper detachment detection method is specified.");
      end
    end

    function visualize(robot)
      robot.graphics_.visualize(robot.LP_, robot.SV_, robot.EE_position_, robot.EE_orientation_dcm_);
    end

    function animation = visualizeForceVectors(robot, animation)
      arguments (Input)
        robot;
        animation (1, 1) {mustBeA(animation, "Animation")};
      end
      [vec_color, vec_width] = animation.getGroundReactionForceVisSettings();

      GRF = robot.SV_.getGroundReactionForce(robot.LP_);

      for limb_id = 1 : robot.LP_.getNumberOfLimb()
        vec_magnitude = GRF(:, limb_id) * animation.getForceExpansionFactor();
        animation.visualizeVector(robot.EE_position_(:, limb_id), ...
          vec_magnitude, vec_color, vec_width);
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function initializeBasePose(robot, config_robot, terrain)
      arguments (Input)
        robot;
        config_robot (1, 1) {mustBeA(config_robot, "ConfigRobot")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end
      initial_base_position_in_Surface = config_robot.getInitialBasePosition();
      robot.base_height_in_Surface_ = initial_base_position_in_Surface(3, 1);
      initial_base_orientation_euler_in_map_frame = config_robot.getInitialBaseOrientationDCM();
      surface_inclination = terrain.getSurfaceInclination();

      ini_base_pos_in_World = rpy2dc(deg2rad(surface_inclination))' * ...
        initial_base_position_in_Surface;
      ini_base_ori_euler_in_World = ...
        -dc2rpy(rpy2dc(deg2rad(initial_base_orientation_euler_in_map_frame))' * ...
        rpy2dc(deg2rad(surface_inclination))');
      ini_base_ori_dcm_in_World = rpy2dc(ini_base_ori_euler_in_World)';

      robot.SV_.setBasePosition(ini_base_pos_in_World);
      robot.SV_.setBaseOrientationDCM(ini_base_ori_dcm_in_World);
      robot.SV_.setBaseOrientationEuler(ini_base_ori_euler_in_World);
    end

    function initializeEEPosition(robot, config_robot, terrain)

      kNumLimb = robot.LP_.getNumberOfLimb();
      desired_initial_EE_distance_xy_from_base_CoM = config_robot.getInitialEEDistXYFromBaseCoM();
      EE_dist_x = desired_initial_EE_distance_xy_from_base_CoM(1, 1);
      EE_dist_y = desired_initial_EE_distance_xy_from_base_CoM(2, 1);
      EE_position_in_base_frame = zeros(3, kNumLimb);
      sign_xy = sign(robot.LP_.c0(1:2, robot.LP_.S0 == 1));
      for i = 1 : kNumLimb
        EE_position_in_base_frame(:, i) = robot.SV_.A0 * [sign_xy(1, i) * EE_dist_x;
                                                        sign_xy(2, i) * EE_dist_y;
                                                        -robot.SV_.R0(3, 1)];
      end
      desired_EE_position_in_inertia_frame = robot.SV_.R0 + EE_position_in_base_frame;

      EE_position = zeros(3, kNumLimb);
      graspable_points = terrain.getGraspablePoints();
      for i = 1 : kNumLimb
        EE_position(:, i) = graspable_points.getNearestPoint( ...
          desired_EE_position_in_inertia_frame(:, i));
      end
      robot.EE_position_ = EE_position;
    end

  end

  %% Setter
  methods (Access = public)

    % For state variables
    function overwriteStateVariables(robot, state_variables)
      arguments (Input)
        robot;
        state_variables (1, 1) {mustBeA(state_variables, "struct")};
      end
      robot.SV_.overwrite(state_variables);
    end

    function setJointTorque(robot, joint_torque)
      arguments (Input)
        robot;
        joint_torque (:, 1) {mustBeA(joint_torque, "double")};
      end
      if (length(joint_torque) ~= robot.LP_.getNumberOfJoints())
        error("ERROR: Failed to set joint torque. " + ...
          "Number of joint torques has to be same as number of joints.");
      end
      robot.SV_.setJointTorque(joint_torque);
    end

    % For desired state variables
    function overwriteDesiredStateVariables(robot, desired_state_variables)
      arguments (Input)
        robot;
        desired_state_variables (1, 1) {mustBeA(desired_state_variables, "struct")};
      end
      robot.des_SV_.overwrite(desired_state_variables);
    end

  end

  %% Getter
  methods (Access = public)

    function type = getType(robot)
      type = robot.kType_;
    end

    function link_parameter = getLinkParameter(robot)
      link_parameter = robot.LP_;
    end

    function state_variable = getStateVariable(robot)
      state_variable = robot.SV_;
    end

    function EE_position = getEEPosition(robot, xyz, limb_id)
      arguments (Input)
        robot;
        xyz (:, 1) uint8 = uint8.empty;
        limb_id (:, 1) uint8 = uint8.empty;
      end
      if (isempty(xyz) && isempty(limb_id))
        xyz = 1 : size(robot.EE_position_, 1);
        limb_id = 1 : size(robot.EE_position_, 2);
      elseif ((isempty(xyz) || isempty(limb_id)))
        error("ERROR: Need to input both of ""xyz"" and ""limb_id"" " + ...
          "if you want to get component of ""EE_position"".");
      elseif (any(xyz < 1) || any(xyz > size(robot.EE_position_, 1)))
        error("ERROR: First input ""xyz"" must be greater than or equal 1 and " + ...
          "less than or equal 3.");
      % elseif (length(xyz) > size(robot.EE_position, 1))
      %   error("ERROR: Dimensional length of ""xyz"" must be under 3.");
      elseif (limb_id > size(robot.EE_position_, 2) ...
          || limb_id(1, 1) < 1 || limb_id(end, 1) > size(robot.EE_position_, 2))
        error("ERROR: Second input ""limb_id"" must be greater than or equal 1 and " + ...
          "less than or equal number of limbs.");
      % elseif (length(limb_id) > size(robot.EE_position, 2))
      %   error("ERROR: Dimensional length of ""limb_id"" must be under number of limbs.");
      end
      EE_position = robot.EE_position_(xyz, limb_id);
    end

    function EE_orientation_dcm = getEEOrientationDCM(robot)
      EE_orientation_dcm = robot.EE_orientation_dcm_;
    end

    function base_height_in_Surface = getBaseHeightInSurfaceFrame(robot)
      base_height_in_Surface = robot.base_height_in_Surface_;
    end

  end

end  % Robot
