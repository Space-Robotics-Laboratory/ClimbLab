classdef RobotGraphics

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    base_graphics_obj_;
    limb_links_graphics_obj_ (:, 3) matlab.graphics.Graphics;
    gripper_graphics_obj_;
    graphics_obj_GRF_vec;
  end
  properties (Access = private)
    kBaseVertices_    (:, 3) double;
    kBaseFaces_       (:, 4) uint8;
    kLimbLinkVertices_ (3, :, :) double;  % (xyz, nx4, kNumLimb)
    kLimbRadius_;
    kLimbColor_;
    kLimbTransparency_
    kGripperVertices_ (8, 3) double;
    kGripperFaces_    (6, 4) uint8;
  end

  %% Public methods
  methods (Access = public)

    function robot_graphics = RobotGraphics(config_robot, LP, SV)
      robot_graphics = robot_graphics.createBaseGraphics(config_robot, LP, SV);
      robot_graphics = robot_graphics.createLimbGraphics(config_robot, LP, SV);
      robot_graphics = robot_graphics.createGripperGraphics();
    end

    function robot_graphics = visualize(robot_graphics, LP, SV, EE_position, EE_orientation_dcm)
      robot_graphics = robot_graphics.visualizeBase(SV);
      robot_graphics = robot_graphics.visualizeLimbs(LP, SV);

      if (LP.getMaxEndurableGrippingForce() == 0.0)
        return;
      end
      robot_graphics = robot_graphics.visualizeGrippers(LP, EE_position, EE_orientation_dcm);
    end

    function deleteGripperGraphics(robot_graphics)
      delete(robot_graphics.gripper_graphics_obj_);
    end

  end

  %% Private Methods
  methods (Access = private)

    function robot_graphics = createBaseGraphics(robot_graphics, config_robot, LP, SV)
      kNumLimb = LP.getNumberOfLimb();
      [base_upper_thickness, base_lower_thickness, base_color, base_alpha] = ...
        config_robot.getBaseVisualSettings();
      if (kNumLimb == 1)
      else
        % Create robot base
        j = LP.getNumberOfJointsPerLimb();
        c0 = LP.getPositionVectorFromBaseCoMToJoint();

        base_upper_V = zeros(kNumLimb, 3);
        base_lower_V = zeros(kNumLimb, 3);
        for i = 1 : kNumLimb
          base_upper_V(i, :) = [c0(1 : 2, j(1, i) * (i - 1) + 1)',  base_upper_thickness];
          base_lower_V(i, :) = [c0(1 : 2, j(1, i) * (i - 1) + 1)', -base_lower_thickness];
        end
        robot_graphics.kBaseVertices_ = [base_upper_V; base_lower_V];

        robot_graphics.kBaseFaces_(1, 1 : kNumLimb) = 1 : kNumLimb;
        robot_graphics.kBaseFaces_(2, 1 : kNumLimb) = kNumLimb + 1 : kNumLimb * 2;
        for i = 1 : kNumLimb
          if (i ~= kNumLimb)
            robot_graphics.kBaseFaces_(i + 2, :) = [i, i + 1, i + 1 + kNumLimb, i + kNumLimb];
          else
            robot_graphics.kBaseFaces_(i + 2, :) = [i, 1, 1 + kNumLimb, i + kNumLimb];
          end
        end
      end

      current_base_position = SV.getBasePosition();
      current_base_orientation_dcm = SV.getBaseOrientationDCM();

      current_base_vertices = robot_graphics.kBaseVertices_ * current_base_orientation_dcm' + ...
        ones(size(robot_graphics.kBaseVertices_, 1), 1) * current_base_position';

      robot_graphics.base_graphics_obj_ = patch( ...
        'Vertices', current_base_vertices, ...
        'Faces', robot_graphics.kBaseFaces_, ...
        'FaceColor', base_color, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', base_alpha, ...
        'Visible', "off");
    end

    function robot_graphics = createLimbGraphics(robot_graphics, config_robot, LP, SV)
      BB = LP.getLinkConnectionRelationship();
      SE = LP.getEndLink();
      cc = LP.getPositionVectorFromLinkCoMToJoint();
      ce = LP.getPositionVectorFromEndLinkCoMToEndPoint();
      [link_radius, limb_color, limb_alpha] = config_robot.getLimbVisualSettings();
      joints = 1 : LP.getNumberOfJoints();
      [joint_position, joint_orientation] = f_kin_j(LP, SV, joints);
      r = link_radius;
      n = 7;
      closed = 1;
      lines = 0;
      k = 1;
      robot_graphics.kLimbLinkVertices_ = zeros(3, n * 4, length(joints));
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

        robot_graphics.limb_links_graphics_obj_(i, 1:3) = [cylinder, end_plate_1, end_plate_2];

        limb_link_vertices_x = [robot_graphics.limb_links_graphics_obj_(i, 1).XData, ...
                                robot_graphics.limb_links_graphics_obj_(i, 2).Vertices(:, 1), ...
                                robot_graphics.limb_links_graphics_obj_(i, 3).Vertices(:, 1)];
        limb_link_vertices_y = [robot_graphics.limb_links_graphics_obj_(i, 1).YData, ...
                                robot_graphics.limb_links_graphics_obj_(i, 2).Vertices(:, 2), ...
                                robot_graphics.limb_links_graphics_obj_(i, 3).Vertices(:, 2)];
        limb_link_vertices_z = [robot_graphics.limb_links_graphics_obj_(i, 1).ZData, ...
                                robot_graphics.limb_links_graphics_obj_(i, 2).Vertices(:, 3), ...
                                robot_graphics.limb_links_graphics_obj_(i, 3).Vertices(:, 3)];
        robot_graphics.kLimbLinkVertices_(:, :, i) = [reshape(limb_link_vertices_x, 1, []);
          reshape(limb_link_vertices_y, 1, []);
          reshape(limb_link_vertices_z, 1, [])];

        current_limb_link_vertices = ...
          joint_orientation(:, 3*i-2:3*i) * robot_graphics.kLimbLinkVertices_(:, :, i) + ...
          joint_position(:, i);

        current_limb_link_vertices_x = reshape(current_limb_link_vertices(1, :), [], 4);
        current_limb_link_vertices_y = reshape(current_limb_link_vertices(2, :), [], 4);
        current_limb_link_vertices_z = reshape(current_limb_link_vertices(3, :), [], 4);

        robot_graphics.limb_links_graphics_obj_(i, 1).XData = current_limb_link_vertices_x(:, 1:2);
        robot_graphics.limb_links_graphics_obj_(i, 1).YData = current_limb_link_vertices_y(:, 1:2);
        robot_graphics.limb_links_graphics_obj_(i, 1).ZData = current_limb_link_vertices_z(:, 1:2);
        robot_graphics.limb_links_graphics_obj_(i, 2).Vertices = [current_limb_link_vertices_x(:, 3), ...
                                                        current_limb_link_vertices_y(:, 3), ...
                                                        current_limb_link_vertices_z(:, 3)];
        robot_graphics.limb_links_graphics_obj_(i, 3).Vertices = [current_limb_link_vertices_x(:, 4), ...
                                                        current_limb_link_vertices_y(:, 4), ...
                                                        current_limb_link_vertices_z(:, 4)];

        robot_graphics.kLimbRadius_ = link_radius;
        robot_graphics.kLimbColor_ = limb_color;
        robot_graphics.kLimbTransparency_ = limb_alpha;
      end
    end

    function robot_graphics = createGripperGraphics(robot_graphics)
      % Create gripper if robot has grippers
      kFingerLength = 1.75 * robot_graphics.kLimbRadius_;
      kFingerThickness = 0.75 * robot_graphics.kLimbRadius_;
      robot_graphics.kGripperVertices_ = [
        -kFingerLength,  kFingerThickness / 2,  0.8 * kFingerThickness;
         kFingerLength,  kFingerThickness / 2,  0.8 * kFingerThickness;
         kFingerLength, -kFingerThickness / 2,  0.8 * kFingerThickness;
        -kFingerLength, -kFingerThickness / 2,  0.8 * kFingerThickness;
        -kFingerLength,  kFingerThickness / 2, -0.2 * kFingerThickness;
         kFingerLength,  kFingerThickness / 2, -0.2 * kFingerThickness;
         kFingerLength, -kFingerThickness / 2, -0.2 * kFingerThickness;
        -kFingerLength, -kFingerThickness / 2, -0.2 * kFingerThickness];
      robot_graphics.kGripperFaces_ = [ 1, 2, 3, 4;
                                        1, 2, 6, 5;
                                        2, 3, 7, 6;
                                        3, 4, 8, 7;
                                        1, 4, 8, 5;
                                        5, 6, 7, 8];
    end

    function robot_graphics = visualizeBase(robot_graphics, SV)
      current_base_position = SV.getBasePosition();
      current_base_orientation_dcm = SV.getBaseOrientationDCM();

      current_base_vertices = robot_graphics.kBaseVertices_ * current_base_orientation_dcm' + ...
        ones(size(robot_graphics.kBaseVertices_, 1), 1) * current_base_position';

      robot_graphics.base_graphics_obj_.Vertices = current_base_vertices;
      robot_graphics.base_graphics_obj_.Visible = "on";
    end

    function robot_graphics = visualizeLimbs(robot_graphics, LP, SV)
      joints = 1 : LP.getNumberOfJoints();
      [joint_position, joint_orientation] = f_kin_j(LP, SV, joints);
      for i = joints
        current_limb_link_vertices = ...
          joint_orientation(:, 3*i-2:3*i) * robot_graphics.kLimbLinkVertices_(:, :, i) + ...
          joint_position(:, i);

        current_limb_link_vertices_x = reshape(current_limb_link_vertices(1, :), [], 4);
        current_limb_link_vertices_y = reshape(current_limb_link_vertices(2, :), [], 4);
        current_limb_link_vertices_z = reshape(current_limb_link_vertices(3, :), [], 4);

        robot_graphics.limb_links_graphics_obj_(i, 1).XData = current_limb_link_vertices_x(:, 1:2);
        robot_graphics.limb_links_graphics_obj_(i, 1).YData = current_limb_link_vertices_y(:, 1:2);
        robot_graphics.limb_links_graphics_obj_(i, 1).ZData = current_limb_link_vertices_z(:, 1:2);
        robot_graphics.limb_links_graphics_obj_(i, 2).Vertices = [current_limb_link_vertices_x(:, 3), ...
                                                        current_limb_link_vertices_y(:, 3), ...
                                                        current_limb_link_vertices_z(:, 3)];
        robot_graphics.limb_links_graphics_obj_(i, 3).Vertices = [current_limb_link_vertices_x(:, 4), ...
                                                        current_limb_link_vertices_y(:, 4), ...
                                                        current_limb_link_vertices_z(:, 4)];

        robot_graphics.limb_links_graphics_obj_(i, 1).Visible = "on";
        robot_graphics.limb_links_graphics_obj_(i, 2).Visible = "on";
        robot_graphics.limb_links_graphics_obj_(i, 3).Visible = "on";
      end
    end

    function robot_graphics = visualizeGrippers(robot_graphics, LP, EE_position, EE_orientation_dcm)
      kNumLimb = LP.getNumberOfLimb();
      current_gripper_vertices = zeros(8, 3, kNumLimb, 2);

      for g = 1 : kNumLimb
        % Rotation matrix to direct the z-direction of the gripper frame in the longitudinal
        % direction of the link
        % NOTE: If visualization of gripper orientation is not work well, you need to check
        % definition of the end-effector frame in LP file
        rot_grip = EE_orientation_dcm(:, 3*g-2:3*g) * rpy2dc([0; pi/2; 0]);
        current_gripper_vertices(:, :, g, 1) = robot_graphics.kGripperVertices_ * rot_grip' ...
          + ones(size(robot_graphics.kGripperVertices_, 1), 1) * EE_position(:, g)';
        current_gripper_vertices(:, :, g, 2) = robot_graphics.kGripperVertices_ * rpy2dc([0; 0; pi/2]) * ...
          rot_grip' ...
          + ones(size(robot_graphics.kGripperVertices_, 1), 1) * EE_position(:, g)';

        robot_graphics.gripper_graphics_obj_(1, g) = patch(...
          'Vertices', current_gripper_vertices(:, :, g, 1), ...
          'Faces', robot_graphics.kGripperFaces_, ...
          'FaceColor', robot_graphics.kLimbColor_, ...
          'EdgeColor', 'none', ...
          'FaceAlpha', robot_graphics.kLimbTransparency_);

        robot_graphics.gripper_graphics_obj_(2, g) = patch(...
          'Vertices', current_gripper_vertices(:, :, g, 2), ...
          'Faces', robot_graphics.kGripperFaces_, ...
          'FaceColor', robot_graphics.kLimbColor_, ...
          'EdgeColor', 'none', ...
          'FaceAlpha', robot_graphics.kLimbTransparency_);
      end
    end

  end

end  % RobotGraphics
