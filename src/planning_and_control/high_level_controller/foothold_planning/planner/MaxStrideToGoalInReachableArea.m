classdef MaxStrideToGoalInReachableArea < handle
% Foothold planning method.
% Select the next swing limb numbers based on the periodic gait sequence
% and foothold positions based on the moving direction and graspable points.
%
% Created     : 2020.04.13 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  properties (SetAccess = private, GetAccess = public)
    output_ FootholdPlannerOutput;

    kinematic_feasibility_is_OK_ (1, 1) logical;

    number_of_graspable_points_in_forward_area_
  end

  %% Methods called only from FootholdPlanning
  methods (Access = {?FootholdPlanning})

    function foothold_planner = MaxStrideToGoalInReachableArea(kNumLimb)
    % Constructor
      arguments (Input)
        kNumLimb (1, 1) {mustBeA(kNumLimb, "uint8")};
      end

      foothold_planner.output_ = FootholdPlannerOutput(kNumLimb);
    end

    function plan(foothold_planner, robot, path_planning, graspable_points_in_reachable_area, kAllowableMaxStride)
      graspable_points_position_in_reachable_area_for_all_limb = graspable_points_in_reachable_area.getPointCloud();
      % while (~foothold_planner.kinematic_feasibility_is_OK_)
        foothold_planner.selectNextSwingLimbId(robot, path_planning, graspable_points_position_in_reachable_area_for_all_limb, kAllowableMaxStride);
      % end
    end

  end

  %% Private Methods
  methods(Access = private)

    function selectNextSwingLimbId(foothold_planner, robot, path_planning, graspable_points_in_reachable_area_for_all_limb, kAllowableMaxStride)
    % Update swing limb ID based on how many graspable points that the swing limb has in its reachable area in forward based on the moving direction from the current end-effector position
    % You can see the details of the planning method is shown in the following paper:
    % --------------------------------------------------------------------
    % In: Proceedings of the International Symposium on Artificial Intelligence Robotics and Automation in Space Integration (iSAIRAS) 2020 by K. Uno et al.
    % Proceedings Paper URL: https://www.hou.usra.edu/meetings/isairas2020fullpapers/pdf/5027.pdf
    % --------------------------------------------------------------------
    % Created     : 2020.07.02 by Kentaro Uno
    % Last updated: 2025.01.09 by Masazumi Imai
    %
    % Input - graspable_points_in_reachable_area_for_all_limb: (3 x n*kNumLimb) (n is number of all graspable points in map)
    %       - kAllowableMaxStride: (1 x 1)
      % arguments (Input)
      %   foothold_planner;
      %   graspable_points_in_reachable_area_for_all_limb
      % end

      kNumLimb = robot.getLinkParameter().getNumberOfLimb();
      current_end_effector_position = robot.getEEPosition();
      moving_direction_unit_vector = path_planning.getLocalPath().getMovingDirection();  % (3 x 1)
      graspable_points_in_reachable_area = reshape(graspable_points_in_reachable_area_for_all_limb, 3, [], kNumLimb);  % (3 x n x kNumLimb)
      number_of_graspable_points = size(graspable_points_in_reachable_area, 2);

      % Initialize variables
      vec_EE_to_GPinRA = NaN(size(graspable_points_in_reachable_area));
      dist_from_EE_to_GPinRA = NaN(1, number_of_graspable_points, double(kNumLimb));
      GP_id_in_max_stride = false(1, number_of_graspable_points, double(kNumLimb));
      graspable_points_in_max_stride = NaN(size(graspable_points_in_reachable_area));
      vec_EE_to_GPinRA_in_max_stride = NaN(size(graspable_points_in_reachable_area));
      inner_product_move_dir_and_EE2GP = NaN(1, number_of_graspable_points, double(kNumLimb));
      GP_id_forward_area = false(1, number_of_graspable_points, double(kNumLimb));
      graspable_points_in_forward_area = NaN(size(graspable_points_in_reachable_area));
      num_of_GP_in_forward_area = zeros(kNumLimb, 1);

      for limb_id = 1 : kNumLimb
        vec_EE_to_GPinRA(:, :, limb_id) = graspable_points_in_reachable_area(:, :, limb_id) - current_end_effector_position(:, limb_id);
        dist_from_EE_to_GPinRA(1, :, limb_id) = vecnorm(vec_EE_to_GPinRA(:, :, limb_id));

        GP_id_in_max_stride(1, :, limb_id) = dist_from_EE_to_GPinRA(:, :, limb_id) <= kAllowableMaxStride;
        graspable_points_in_max_stride(:, GP_id_in_max_stride(1, :, limb_id), limb_id) = graspable_points_in_reachable_area(:, GP_id_in_max_stride(1, :, limb_id), limb_id);

        vec_EE_to_GPinRA_in_max_stride(:, GP_id_in_max_stride(1, :, limb_id), limb_id) = vec_EE_to_GPinRA(:, GP_id_in_max_stride(1, :, limb_id), limb_id);

        % Inner product of moving direction vector and vec_EE_to_GPinRA_in_max_stride
        inner_product_move_dir_and_EE2GP(1, :, limb_id) = moving_direction_unit_vector' * vec_EE_to_GPinRA_in_max_stride(:, :, limb_id);

        % Graspable points in the forward area based on the moving direction from the current end-effector position
        GP_id_forward_area(1, :, limb_id) = inner_product_move_dir_and_EE2GP(1, :, limb_id) > 0.0;
        graspable_points_in_forward_area(:, GP_id_forward_area(1, :, limb_id), limb_id) = graspable_points_in_max_stride(:, GP_id_forward_area(1, :, limb_id), limb_id);

        num_of_GP_in_forward_area(limb_id, 1) = sum(GP_id_forward_area(1, :, limb_id) == true);
      end

      if (all(num_of_GP_in_forward_area == 0))
        error("ERROR: Failed to select next swing limb ID." + newline + ...
          "All limbs has no kinematically feasible graspable points.");
      end

      [~, limb_id_with_max_num_GP] = max(num_of_GP_in_forward_area);
      next_swing_limb_id = uint8(limb_id_with_max_num_GP);

      foothold_planner.output_.setSwingLimbId(next_swing_limb_id);
    end

    function selectNextGraspingPoint(foothold_planner)
    end

    function updateFootholdPositions(foothold_planner, terrain, path_planning, foothold_planning)
    end

  end

  %% Getter
  methods (Access = public)

    function output = getOutput(foothold_planner)
      output = foothold_planner.output_;
    end

  end

end  % MaxStrideToGoalInReachableArea
