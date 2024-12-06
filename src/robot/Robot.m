classdef Robot
  %% Properties
  properties (SetAccess = private, GetAccess = public)
    type (1, 1) string;
    LP LinkParameters;
    SV StateVariable;
    des_SV StateVariable;

    kinematics Kinematics;
    dynamics Dynamics;

    EE_position (3, :) double;
    EE_orientation_dcm (3, :) double;
    % EE_orientation_euler_rad (3, :) double;
    % EE_orientation_euler_deg (3, :) double;

    base_height_in_Surface (1, 1) double;

    contact_state ContactState;

    is_supporting (1, :) logical;
    desired_support_limb_id (1, :) logical;
    gripper_detachment_detection_method (1, 1) string;
    is_grasping (1, :) logical;
    is_slipping (1, :) logical;

    graphics_obj_base;
    graphics_obj_limb_links (:, 3) matlab.graphics.Graphics;
    graphics_obj_robot_gripper;
    graphics_obj_GRF_vec;
  end
  properties (Access = private)
    base_vertices    (:, 3) double;
    base_faces       (:, 4) uint8;
    limb_link_vertices (3, :, :) double;  % (xyz, nx4, num_limb)
    gripper_vertices (8, 3) double;
    gripper_faces    (6, 4) uint8;
  end

  %% Public methods
  methods (Access = public)

    function robot = Robot(config, world, terrain)
    % Robot() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigRobot")};
        world (1, 1) {mustBeA(world, "World")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end
      robot.type = config.getRobotType();
      robot.LP = LinkParameters(robot.type + "_LP");

      robot.SV = StateVariable(robot.LP.getNumberOfJoints());
      robot = robot.initializeBasePose(config, terrain);
      robot.des_SV = StateVariable(robot.LP.getNumberOfJoints());

      num_limb = robot.LP.getNumberOfLimb();
      robot.EE_position = zeros(3, num_limb);
      robot.EE_position = robot.initializeEEPosition(config, terrain);
      robot.EE_orientation_dcm = zeros(3, 3 * num_limb);
      % robot.EE_orientation_euler_rad = zeros(3, num_limb);
      % robot.EE_orientation_euler_deg = zeros(3, num_limb);

      robot.kinematics = Kinematics(robot);
      base_position = robot.SV.getBasePosition();
      base_orientation_dcm = robot.SV.getBaseOrientationDCM();
      joint_angles = robot.kinematics.computeInverse(base_position, base_orientation_dcm, ...
        robot.getEEPosition());
      robot.SV = robot.SV.setJointAnglularPositions(joint_angles);
      robot.SV = robot.SV.calcLinkPose(robot.LP);
      robot = robot.forwardKinematics();

      robot.dynamics = Dynamics(world.getUseDynamics());

      robot.des_SV = robot.des_SV.overwrite(robot.SV.clone());

      robot.contact_state = ContactState(num_limb);

      robot.is_supporting = false(1, num_limb);
      robot.desired_support_limb_id = true(1, num_limb);
      robot.gripper_detachment_detection_method = config.getGripperDetachmentDetectionMethod();
      robot.is_grasping = false(1, num_limb);
      robot.is_slipping = false(1, num_limb);
      robot.contact_state = robot.contact_state.detectEECollision(robot, terrain);

      robot = robot.updateGripperState(terrain);

      if (~config.getVisualizeRobot())
        return;
      end
      robot = robot.createRobotBaseModel(config);
      robot = robot.createRobotLimbModel(config);
      if (robot.LP.getMaxEdurableGrippingForce() > 0.0)
        robot = robot.createRobotGripperModel(config);
      end
    end

    function robot = forwardKinematics(robot)
    % forwardKinematics()
    %   Compute forward kinematics
      [robot.EE_position, robot.EE_orientation_dcm] = robot.kinematics.computeForward( ...
        robot.LP, robot.SV);
    end

    function robot = forwardDynamics(robot)
    % forwardDynamics()
    %   Compute forward dynamics
      robot = robot.dynamics.computeForward(robot);
    end

    function robot = detectCollision(robot, terrain)
    % detectCollision()
    %   Detect a new contact (or losing an old one) between the robot end-effector and the ground
    %   surface, and save contact pose of End-Effectors.
      arguments (Input)
        robot;
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end
      robot.contact_state = robot.contact_state.detectEECollision(robot, terrain);
    end

    function robot = calcGraundReactionForces(robot, terrain)
    % calcGraundReactionForces()
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

      num_limb = robot.LP.getNumberOfLimb();
      num_joints = robot.LP.getNumberOfJoints();
      [~, EndEffectors] = find(robot.LP.getEndLink() == 1);
      EE_in_contact = robot.contact_state.getInContact();
      contact_position = robot.contact_state.getPosition();
      [Kf, Df, Km, Dm] = terrain.getGroundCoefficients();

      GJ = zeros(6, num_joints, num_limb);
      GRF = zeros(3, robot.LP.getNumberOfJoints());

      for limb_id = 1 : num_limb
        EE = EndEffectors(1, limb_id);

        if (robot.is_grasping(1, limb_id) || EE_in_contact(1, limb_id))
          GJ(:, :, limb_id) = calc_gj(robot.LP.clone(), robot.SV.clone(), limb_id);
          EE_velocity(:, limb_id) = GJ(:, :, limb_id) * robot.SV.getJointAngularVelocity();

          GRF(:, EE) = - Kf * (robot.EE_position(:, limb_id) - contact_position(:, limb_id)) - ...
            Df * EE_velocity(1:3, limb_id);
        else
          GRF(:, EE) = zeros(3, 1);
        end
      end
      robot.SV = robot.SV.applyExternalForces(GRF);
    end

    function robot = updateDesiredGripperState(robot, ...
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

      num_limb = robot.LP.getNumberOfLimb();
      swing_limb_id = foothold_planning.getSwingLimbID();

      for limb_id = 1 : num_limb
        if (all(limb_id ~= swing_limb_id))
          continue;
        end

        swing_timings = gait_planning.getSwingTimings();
        landing_timings = gait_planning.getLandingTimings();

        % Release gripper of swing limb at swing motion start time
        if (abs(time - swing_timings(1, limb_id)) < eps)
          robot.desired_support_limb_id(1, limb_id) = false;
        end

        % Determining whether to close the gripper
        if (time < (swing_timings(1, limb_id) + landing_timings(1, limb_id)) / 2 ...
            || robot.desired_support_limb_id(1, limb_id) ~= false)
          continue;
        end

        swing_EE_position = robot.EE_position(:, limb_id);
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
          robot.desired_support_limb_id(1, limb_id) = true;
        end
      end

      % Store last end-effector positions
      EE_position_last = robot.EE_position;
    end

    function robot = updateGripperState(robot, terrain)
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
      switch (robot.gripper_detachment_detection_method)
        case "none"
        case "max_holding_force"
          num_limb = robot.LP.getNumberOfLimb();
          ground_reaction_force = robot.SV.getGroundReactionForce(robot.LP);
          F_grip = robot.LP.getMaxEdurableGrippingForce();

          robot.contact_state = robot.contact_state.detectEECollision(robot, terrain);
          EE_in_contact = robot.contact_state.getInContact();
          contact_EE_position = robot.contact_state.getPosition();
          contact_EE_orientation_dcm = robot.contact_state.getOrientationDCM();

          for limb_id = 1 : num_limb
            % Swing limb EE is not grasping and does not cause slip
            if (~robot.desired_support_limb_id(1, limb_id))
              robot.is_supporting(1, limb_id) = false;
              robot.is_grasping(1, limb_id) = false;
              robot.is_slipping(1, limb_id) = false;
              if (~EE_in_contact(1, limb_id))
                contact_EE_position(:, limb_id) = NaN(3, 1);
                contact_EE_orientation_dcm(:, 3*limb_id-2 : 3*limb_id) = NaN;
              end
              continue;
            end
            % Gripper detachment is NOT caused
            if (norm(ground_reaction_force(:, limb_id)) <= F_grip || EE_in_contact(1, limb_id))
              if (~robot.is_supporting(1, limb_id))
                robot.is_supporting(1, limb_id) = true;
                contact_EE_position(:, limb_id) = ...
                  terrain.getNearestPointInWorldFrame(robot.EE_position(:, limb_id));
                contact_EE_orientation_dcm(:, 3*limb_id-2 : 3*limb_id) = ...
                  robot.EE_orientation_dcm(:, 3*limb_id-2 : 3*limb_id);
              end
              robot.is_grasping(1, limb_id) = true;
              robot.is_slipping(1, limb_id) = false;
            % Gripper detachment is caused when the Ground Reaction Force acting on End-Effector in
            % the pulling direction exceeded the maximum tolerable grasping force
            else
              robot.is_supporting(1, limb_id) = false;
              robot.is_grasping(1, limb_id) = false;
              robot.is_slipping(1, limb_id) = true;
            end
          end
          robot.contact_state = robot.contact_state.setContactPose( ...
            contact_EE_position, contact_EE_orientation_dcm);
        otherwise
          error("ERROR: Invalid gripper detachment detection method is specified.");
      end

      % If the gripper of support limb slips, that limb is not a support limb
      % for limb_id = 1 :num_limb
      %   if (robot.is_slipping(1, limb_id))
      %     robot.is_supporting(1, limb_id) = false;
      %   end
      % end
    end

    function robot = visualize(robot, config)
      robot = robot.visualizeBase();
      robot = robot.visualizeLimbs();
      if (robot.LP.getMaxEdurableGrippingForce() == 0.0)
        return;
      end
      robot = robot.visualizeGrippers(config);
    end

    function animation = visualizeForceVectors(robot, animation)
      arguments (Input)
        robot;
        animation (1, 1) {mustBeA(animation, "Animation")};
      end
      [vec_color, vec_width] = animation.getGroundReactionForceVisSettings();

      GRF = robot.SV.getGroundReactionForce(robot.LP);

      for limb_id = 1 : robot.LP.getNumberOfLimb()
        vec_magnitude = GRF(:, limb_id) * animation.getForceExpansionFactor();
        animation = animation.visualizeVector(robot.EE_position(:, limb_id), ...
          vec_magnitude, vec_color, vec_width);
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function robot = initializeBasePose(robot, config, terrain)
      arguments (Input)
        robot;
        config (1, 1) {mustBeA(config, "ConfigRobot")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end
      initial_base_position_in_Surface = config.getInitialBasePosition();
      robot.base_height_in_Surface = initial_base_position_in_Surface(3, 1);
      initial_base_orientation_euler_in_map_frame = config.getInitialBaseOrientationDCM();
      surface_inclination = terrain.getSurfaceInclination();

      ini_base_pos_in_World = rpy2dc(deg2rad(surface_inclination))' * ...
        initial_base_position_in_Surface;
      ini_base_ori_euler_in_World = ...
        -dc2rpy(rpy2dc(deg2rad(initial_base_orientation_euler_in_map_frame))' * ...
        rpy2dc(deg2rad(surface_inclination))');
      ini_base_ori_dcm_in_World = rpy2dc(ini_base_ori_euler_in_World)';

      robot.SV = robot.SV.setBasePosition(ini_base_pos_in_World);
      robot.SV = robot.SV.setBaseOrientationDCM(ini_base_ori_dcm_in_World);
      robot.SV = robot.SV.setBaseOrientationEuler(ini_base_ori_euler_in_World);
    end

    function EE_position = initializeEEPosition(robot, config, terrain)

      num_limb = robot.LP.getNumberOfLimb();
      desired_initial_EE_distance_xy_from_base_CoM = config.getInitialEEDistXYFromBaseCoM();
      EE_dist_x = desired_initial_EE_distance_xy_from_base_CoM(1, 1);
      EE_dist_y = desired_initial_EE_distance_xy_from_base_CoM(2, 1);
      EE_position_in_base_frame = zeros(3, num_limb);
      sign_xy = sign(robot.LP.c0(1:2, robot.LP.S0 == 1));
      for i = 1:num_limb
        EE_position_in_base_frame(:, i) = robot.SV.A0 * [sign_xy(1, i) * EE_dist_x;
                                                        sign_xy(2, i) * EE_dist_y;
                                                        -robot.SV.R0(3, 1)];
      end
      desired_EE_position_in_inertia_frame = robot.SV.R0 + EE_position_in_base_frame;

      EE_position = zeros(3, num_limb);
      graspable_points = terrain.getGraspablePoints();
      for i = 1:num_limb
        EE_position(:, i) = graspable_points.getNearestPoint( ...
          desired_EE_position_in_inertia_frame(:, i));
      end
    end

  end

  %% Setter
  methods (Access = public)
    % For state variables
    function robot = overwriteStateVariables(robot, state_variables)
      arguments (Input)
        robot;
        state_variables (1, 1) {mustBeA(state_variables, "struct")};
      end
      robot.SV = robot.SV.overwrite(state_variables);
    end
    function robot = setJointTorque(robot, joint_torque)
      arguments (Input)
        robot;
        joint_torque (:, 1) {mustBeA(joint_torque, "double")};
      end
      if (length(joint_torque) ~= robot.LP.getNumberOfJoints())
        error("ERROR: Failed to set joint torque. " + ...
          "Number of joint torques has to be same as number of joints.");
      end
      robot.SV = robot.SV.setJointTorque(joint_torque);
    end
    % For desired state variables
    function robot = overwriteDesiredStateVariables(robot, desired_state_variables)
      arguments (Input)
        robot;
        desired_state_variables (1, 1) {mustBeA(desired_state_variables, "struct")};
      end
      robot.des_SV = robot.des_SV.overwrite(desired_state_variables);
    end
  end

  %% Getter
  methods (Access = public)
    function type = getType(robot)
      type = robot.type;
    end
    function EE_position = getEEPosition(robot, xyz, limb_id)
      arguments (Input)
        robot;
        xyz (:, 1) uint8 = uint8.empty;
        limb_id (:, 1) uint8 = uint8.empty;
      end
      if (isempty(xyz) && isempty(limb_id))
        xyz = 1 : size(robot.EE_position, 1);
        limb_id = 1 : size(robot.EE_position, 2);
      elseif ((isempty(xyz) || isempty(limb_id)))
        error("ERROR: Need to input both of ""xyz"" and ""limb_id"" " + ...
          "if you want to get componet of ""EE_position"".");
      elseif (any(xyz < 1) || any(xyz > size(robot.EE_position, 1)))
        error("ERROR: First input ""xyz"" must be greater than or equal 1 and " + ...
          "less than or equal 3.");
      % elseif (length(xyz) > size(robot.EE_position, 1))
      %   error("ERROR: Dimensional length of ""xyz"" must be under 3.");
      elseif (limb_id > size(robot.EE_position, 2) ...
          || limb_id(1, 1) < 1 || limb_id(end, 1) > size(robot.EE_position, 2))
        error("ERROR: Second input ""limb_id"" must be greater than or equal 1 and " + ...
          "less than or equal number of limbs.");
      % elseif (length(limb_id) > size(robot.EE_position, 2))
      %   error("ERROR: Dimensional length of ""limb_id"" must be under number of limbs.");
      end
      EE_position = robot.EE_position(xyz, limb_id);
    end
    function EE_orientation_dcm = getEEOrientationDCM(robot)
      EE_orientation_dcm = robot.EE_orientation_dcm;
    end
    function base_height_in_Surface = getBaseHeightInSurfaceFrame(robot)
      base_height_in_Surface = robot.base_height_in_Surface;
    end

    function EE_is_grasping = getEEIsGrasping(robot)
      EE_is_grasping = robot.is_grasping;
    end
  end


  %% Private Methods (for Visualization)
  methods (Access = private)

    function robot = createRobotBaseModel(robot, config)
      num_limb = robot.LP.getNumberOfLimb();
      [base_upper_thickness, base_lower_thickness, base_color, base_alpha] = ...
        config.getBaseVisualSettings();
      if (num_limb == 1)
      else
        % Create robot base
        j = robot.LP.getNumberOfJointsPerLimb();
        c0 = robot.LP.getPositionVectorFromBaseCoMToJoint();

        base_upper_V = zeros(num_limb, 3);
        base_lower_V = zeros(num_limb, 3);
        for i = 1:num_limb
          base_upper_V(i, :) = [c0(1:2, j(1, i)*(i-1)+1)',  base_upper_thickness];
          base_lower_V(i, :) = [c0(1:2, j(1, i)*(i-1)+1)', -base_lower_thickness];
        end
        robot.base_vertices = [base_upper_V; base_lower_V];

        robot.base_faces(1, 1:num_limb) = 1 : num_limb;
        robot.base_faces(2, 1:num_limb) = num_limb + 1 : num_limb * 2;
        for i = 1:num_limb
          if (i ~= num_limb)
            robot.base_faces(i + 2, :) = [i, i + 1, i + 1 + num_limb, i + num_limb];
          else
            robot.base_faces(i + 2, :) = [i, 1, 1 + num_limb, i + num_limb];
          end
        end
      end

      current_base_position = robot.SV.getBasePosition();
      current_base_orientation_dcm = robot.SV.getBaseOrientationDCM();

      current_base_vertices = robot.base_vertices * current_base_orientation_dcm' + ...
        ones(size(robot.base_vertices, 1), 1) * current_base_position';
      current_base_faces = robot.base_faces;

      robot.graphics_obj_base = patch( ...
        'Vertices', current_base_vertices, 'Faces', current_base_faces, ...
        'FaceColor', base_color, 'EdgeColor', 'none', 'FaceAlpha', base_alpha, ...
        'Visible', "off");
    end

    function robot = createRobotLimbModel(robot, config)
      BB = robot.LP.getLinkConnectionRelationship();
      SE = robot.LP.getEndLink();
      cc = robot.LP.getPositionVectorFromLinkCoMToJoint();
      ce = robot.LP.getPositionVectorFromEndLinkCoMToEndPoint();
      [link_radius, limb_color, limb_alpha] = config.getLimbVisualSettings();
      joints = 1 : robot.LP.getNumberOfJoints();
      [joint_position, joint_orientation] = f_kin_j(robot.LP, robot.SV, joints);
      r = link_radius;
      n = 7;
      closed = 1;
      lines = 0;
      k = 1;
      robot.limb_link_vertices = zeros(3, n * 4, length(joints));
      for i = joints
        % Create links, which are NOT connected with the end-effector
        if (SE(1, i) == 0)
          [~, col] = find(BB == i);
          [cylinder, end_plate_1, end_plate_2] = ...
            vis_cylinder(zeros(3, 1), cc(:, i, col) - cc(:, i, i), ...
            r, n, limb_color, limb_alpha, closed, lines);
        % Create links, which are connected with the end-effector
        else
          [cylinder, end_plate_1, end_plate_2] = ...
            vis_cylinder(zeros(3, 1), ce(:, i) - cc(:, i, i), ...
            r, n, limb_color, limb_alpha, closed, lines);
          k = k + 1;
        end
        robot.graphics_obj_limb_links(i, 1:3) = [cylinder, end_plate_1, end_plate_2];

        limb_link_vertices_x = [robot.graphics_obj_limb_links(i, 1).XData, ...
                                robot.graphics_obj_limb_links(i, 2).Vertices(:, 1), ...
                                robot.graphics_obj_limb_links(i, 3).Vertices(:, 1)];
        limb_link_vertices_y = [robot.graphics_obj_limb_links(i, 1).YData, ...
                                robot.graphics_obj_limb_links(i, 2).Vertices(:, 2), ...
                                robot.graphics_obj_limb_links(i, 3).Vertices(:, 2)];
        limb_link_vertices_z = [robot.graphics_obj_limb_links(i, 1).ZData, ...
                                robot.graphics_obj_limb_links(i, 2).Vertices(:, 3), ...
                                robot.graphics_obj_limb_links(i, 3).Vertices(:, 3)];
        robot.limb_link_vertices(:, :, i) = [reshape(limb_link_vertices_x, 1, []);
          reshape(limb_link_vertices_y, 1, []);
          reshape(limb_link_vertices_z, 1, [])];

        current_limb_link_vertices = ...
          joint_orientation(:, 3*i-2:3*i) * robot.limb_link_vertices(:, :, i) + ...
          joint_position(:, i);

        current_limb_link_vertices_x = reshape(current_limb_link_vertices(1, :), [], 4);
        current_limb_link_vertices_y = reshape(current_limb_link_vertices(2, :), [], 4);
        current_limb_link_vertices_z = reshape(current_limb_link_vertices(3, :), [], 4);

        robot.graphics_obj_limb_links(i, 1).XData = current_limb_link_vertices_x(:, 1:2);
        robot.graphics_obj_limb_links(i, 1).YData = current_limb_link_vertices_y(:, 1:2);
        robot.graphics_obj_limb_links(i, 1).ZData = current_limb_link_vertices_z(:, 1:2);
        robot.graphics_obj_limb_links(i, 2).Vertices = [current_limb_link_vertices_x(:, 3), ...
                                                        current_limb_link_vertices_y(:, 3), ...
                                                        current_limb_link_vertices_z(:, 3)];
        robot.graphics_obj_limb_links(i, 3).Vertices = [current_limb_link_vertices_x(:, 4), ...
                                                        current_limb_link_vertices_y(:, 4), ...
                                                        current_limb_link_vertices_z(:, 4)];
      end
    end

    function robot = createRobotGripperModel(robot, config)
      % Create gripper if robot has grippers
      [link_radius, ~, ~] = config.getLimbVisualSettings();
      finger_length = 1.75 * link_radius;
      finger_thickness = 0.75 * link_radius;
      robot.gripper_vertices = [
        -finger_length,  finger_thickness / 2,  0.8 * finger_thickness;
         finger_length,  finger_thickness / 2,  0.8 * finger_thickness;
         finger_length, -finger_thickness / 2,  0.8 * finger_thickness;
        -finger_length, -finger_thickness / 2,  0.8 * finger_thickness;
        -finger_length,  finger_thickness / 2, -0.2 * finger_thickness;
         finger_length,  finger_thickness / 2, -0.2 * finger_thickness;
         finger_length, -finger_thickness / 2, -0.2 * finger_thickness;
        -finger_length, -finger_thickness / 2, -0.2 * finger_thickness];
      robot.gripper_faces = [ 1, 2, 3, 4;  1, 2, 6, 5;  2, 3, 7, 6;  3, 4, 8, 7;
                              1, 4, 8, 5;  5, 6, 7, 8];
    end

    function robot = visualizeBase(robot)
      current_base_position = robot.SV.getBasePosition();
      current_base_orientation_dcm = robot.SV.getBaseOrientationDCM();

      current_base_vertices = robot.base_vertices * current_base_orientation_dcm' + ...
        ones(size(robot.base_vertices, 1), 1) * current_base_position';

      robot.graphics_obj_base.Vertices = current_base_vertices;
      robot.graphics_obj_base.Visible = "on";
    end

    function robot = visualizeLimbs(robot)
      joints = 1 : robot.LP.getNumberOfJoints();
      [joint_position, joint_orientation] = f_kin_j(robot.LP, robot.SV, joints);
      for i = joints
        current_limb_link_vertices = ...
          joint_orientation(:, 3*i-2:3*i) * robot.limb_link_vertices(:, :, i) + ...
          joint_position(:, i);

        current_limb_link_vertices_x = reshape(current_limb_link_vertices(1, :), [], 4);
        current_limb_link_vertices_y = reshape(current_limb_link_vertices(2, :), [], 4);
        current_limb_link_vertices_z = reshape(current_limb_link_vertices(3, :), [], 4);

        robot.graphics_obj_limb_links(i, 1).XData = current_limb_link_vertices_x(:, 1:2);
        robot.graphics_obj_limb_links(i, 1).YData = current_limb_link_vertices_y(:, 1:2);
        robot.graphics_obj_limb_links(i, 1).ZData = current_limb_link_vertices_z(:, 1:2);
        robot.graphics_obj_limb_links(i, 2).Vertices = [current_limb_link_vertices_x(:, 3), ...
                                                        current_limb_link_vertices_y(:, 3), ...
                                                        current_limb_link_vertices_z(:, 3)];
        robot.graphics_obj_limb_links(i, 3).Vertices = [current_limb_link_vertices_x(:, 4), ...
                                                        current_limb_link_vertices_y(:, 4), ...
                                                        current_limb_link_vertices_z(:, 4)];

        robot.graphics_obj_limb_links(i, 1).Visible = "on";
        robot.graphics_obj_limb_links(i, 2).Visible = "on";
        robot.graphics_obj_limb_links(i, 3).Visible = "on";
      end
    end

    function robot = visualizeGrippers(robot, config)
      [~, limb_color, limb_alpha] = config.getLimbVisualSettings();
      num_limb = robot.LP.getNumberOfLimb();
      current_gripper_vertices = zeros(8, 3, num_limb, 2);
      for g = 1:num_limb
        % Rotation matrix to direct the z-direction of the gripper frame in the longitudinal
        % direction of the link
        % NOTE: If visualization of gripper orientation is not work well, you need to check
        % definition of the end-effector frame in LP file
        rot_grip = robot.EE_orientation_dcm(:, 3*g-2:3*g) * rpy2dc([0; pi/2; 0]);
        current_gripper_vertices(:, :, g, 1) = robot.gripper_vertices * rot_grip' ...
          + ones(size(robot.gripper_vertices, 1), 1) * robot.EE_position(:, g)';
        current_gripper_vertices(:, :, g, 2) = robot.gripper_vertices * rpy2dc([0; 0; pi/2]) * ...
          rot_grip' ...
          + ones(size(robot.gripper_vertices, 1), 1) * robot.EE_position(:, g)';

        robot.graphics_obj_robot_gripper(1, g) = patch(...
          'Vertices', current_gripper_vertices(:, :, g, 1), 'Faces', robot.gripper_faces, ...
          'FaceColor', limb_color, 'EdgeColor', 'none', 'FaceAlpha', limb_alpha);
        robot.graphics_obj_robot_gripper(2, g) = patch(...
          'Vertices', current_gripper_vertices(:, :, g, 2), 'Faces', robot.gripper_faces, ...
          'FaceColor', limb_color, 'EdgeColor', 'none', 'FaceAlpha', limb_alpha);
      end
    end

  end

end
% EOF