classdef IntersectionOfDiagonalLineAndMovingDirection
% IntersectionOfDiagonalLines
% Calculate the robot base position based on the intersection of a diagonal line formed by diagonal foothold positions and moving direction vector
%
% Created     : 2024.09.29 by Masazumi Imai
% Last updated: 2024.12.07 by Masazumi Imai
% TODO: Need to be improved (coordinate is not good maybe), refer to IntersectionOfDiagonalLines

  %% Properties
  properties (SetAccess = private, GetAccess = public)
  end

  %% Public Methods
  methods (Access = public)

    function planner = IntersectionOfDiagonalLineAndMovingDirection()
    % IntersectionOfDiagonalLineAndMovingDirection() Constructor
    end

    function desired_base_position = plan(planner, robot, path_planning, foothold_planning)
      arguments (Input)
        planner;
        robot             (1, 1) {mustBeA(robot, "Robot")};
        path_planning     (1, 1) {mustBeA(path_planning, "PathPlanning")};
        foothold_planning (1, 1) {mustBeA(foothold_planning, "FootholdPlanning")};
      end

      current_base_position_in_World = robot.SV.getBasePosition();
      moving_direction_vector = path_planning.local_path.getMovingDirection();
      current_EE_positions_in_World = robot.getEEPosition();
      desired_EE_positions_in_World = foothold_planning.getFootholdPositions();
      swing_limb_id = foothold_planning.getSwingLimbID();

      [diagonal_lines, comb_diag_limb] = planner.calcDiagonalLines( ...
        current_EE_positions_in_World, desired_EE_positions_in_World, swing_limb_id);

      % Diagonal line with swing limb
      [idx, ~] = find(comb_diag_limb == swing_limb_id);
      diagonal_line_sw = diagonal_lines(:, :, comb_diag_limb(idx, 1), comb_diag_limb(idx, 2));

      projction_of_base_position_in_World = [current_base_position_in_World(1:2); 0.0];

      intersection_in_World_xy = planner.calcIntersection( ...
        projction_of_base_position_in_World, moving_direction_vector, diagonal_line_sw);

      desired_base_position = planner.calcDesiredBasePosition( ...
        current_base_position_in_World, projction_of_base_position_in_World, ...
        moving_direction_vector, intersection_in_World_xy);
    end

  end

  %% Private Methods
  methods (Access = private)

    function [diagonal_lines, comb_diag_limb] = calcDiagonalLines(~, ...
        current_EE_positions_in_World, desired_EE_positions_in_World, swing_limb_id)

      num_limb = size(current_EE_positions_in_World, 2);

      % EE2EE_line (3, 2, i, j): Start and end positions of line connecting i-th and j-th EE
      %   EE2EE_line(:, 1, i, j): Position of i-th EE to form a line with j-th EE
      %   EE2EE_line(:, 2, i, j): Position of j-th EE to form a line with i-th EE
      EE2EE_line = zeros(3, 2, num_limb, num_limb);
      % Combination of 2 limbs selected from all limbs
      comb_limbs = nchoosek(1:num_limb, 2);
      % Combination of limbs forming a diagonal line
      comb_diag_limb = [];
      % Count the number of limb combinations that form a diagonal line
      cnt = 1;

      % diagonal_lines(3, 2, i, j): Start and end positions of line connecting i-th and j-th EE
      %   diagonal_lines(:, 1, i, j): Position of i-th EE to form a diagonal line with j-th EE
      %   diagonal_lines(:, 2, i, j): Position of j-th EE to form a diagonal line with i-th EE
      diagonal_lines = zeros(3, 2, num_limb, num_limb);

      % Calculation of lines with limbs that form combinations
      for k = 1 : size(comb_limbs, 1)
        i = comb_limbs(k, 1);
        j = comb_limbs(k, 2);
        % Limb ID before and after i (i-1, i+1)
        if (i == 1)
          i_minus_1 = num_limb;
        else
          i_minus_1 = i - 1;
        end
        i_plus_1 = i + 1;
        % Limb ID before and after j (j-1, j+1)
        if (j == 1)
          j_minus_1 = num_limb;
        else
          j_minus_1 = j - 1;
        end
        if (j == num_limb)
          j_plus_1 = 1;
        else
          j_plus_1 = j + 1;
        end

        % If i is swing limb, EE2EE_line(:, 1, i, j) is next position of swing limb
        if (i == swing_limb_id)
          EE2EE_line(:, 1, i, j) = desired_EE_positions_in_World(:, i);
          EE2EE_line(:, 2, i, j) = current_EE_positions_in_World(:, j);
          % Diagonal line by the i-th limb
          if (j ~= i_minus_1 && j ~= i_plus_1)
            diagonal_lines(:, 1, i, j) = desired_EE_positions_in_World(:, i);
            diagonal_lines(:, 2, i, j) = current_EE_positions_in_World(:, j);
            comb_diag_limb(cnt, :) = [i, j];
            cnt = cnt + 1;
          end
        % If j is swing limb, EE2EE_line(:, 2, i, j) is next position of swing limb
        elseif (j == swing_limb_id)
          EE2EE_line(:, 1, i, j) = current_EE_positions_in_World(:, i);
          EE2EE_line(:, 2, i, j) = desired_EE_positions_in_World(:, j);
          % Diagonal line by the j-th limb
          if (i ~= j_minus_1 && i ~= j_plus_1)
            diagonal_lines(:, 1, i, j) = current_EE_positions_in_World(:, i);
            diagonal_lines(:, 2, i, j) = desired_EE_positions_in_World(:, j);
            comb_diag_limb(cnt, :) = [i, j];
            cnt = cnt + 1;
          end
        % If i and j are supporting limb
        else
          EE2EE_line(:, 1, i, j) = current_EE_positions_in_World(:, i);
          EE2EE_line(:, 2, i, j) = current_EE_positions_in_World(:, j);
          % Diagonal line by the i-th limb
          if (j ~= i_minus_1 && j ~= i_plus_1)
            diagonal_lines(:, 1, i, j) = current_EE_positions_in_World(:, i);
            diagonal_lines(:, 2, i, j) = current_EE_positions_in_World(:, j);
            comb_diag_limb(cnt, :) = [i, j];
            cnt = cnt + 1;
          end
        end
      end
    end

    function intersection_in_World_xy = calcIntersection(~, ...
        projction_of_base_position_in_World, moving_direction_vector, diagonal_line_sw)

      % projction_of_base_position_in_World = [current_base_position_in_World(1:2); 0.0];

      %%% Vectors needed to calculate intersection
      % Vector of diagonal line projected in x-y plane of World frame
      vec_diag_sw_proj = [diagonal_line_sw(1:2, 2) - diagonal_line_sw(1:2, 1); 0.0];
      % Vector of moving direction projected in x-y plane of World frame
      vec_mov_proj = [moving_direction_vector(1:2); 0.0];
      % Vector from stat point of "vec_diag_sw_proj" to "projction_of_base_position_in_World"
      vec_s2s = projction_of_base_position_in_World - [diagonal_line_sw(1:2, 1); 0.0];

      %%% Calculate intersection of a diagonal line and moving direction vector
      % Ratio from start point of "vec_bg_wrt_inertial_frame" to the intersection
      ratio = - cross(vec_diag_sw_proj, vec_mov_proj) \ cross(vec_diag_sw_proj, vec_s2s);
      intersection_in_World_xy = projction_of_base_position_in_World + vec_mov_proj * ratio;
    end

    function desired_base_position = calcDesiredBasePosition(~, ...
        current_base_position_in_World, projction_of_base_position_in_World, ...
        moving_direction_vector, intersection_in_World_xy)

      % NOTE: desired_base_position should be calculated by dot product?
      vec_a = projction_of_base_position_in_World + moving_direction_vector;
      vec_b = intersection_in_World_xy - projction_of_base_position_in_World;
      % Elevation of moving direction vector with respect to x-y plane of World
      elevation_rad = acos( dot(vec_a, vec_b) / (norm(vec_a) * norm(vec_b)) );
      % Vector from current base position to desired base position
      vec_base_cur_to_base_des = ...
        norm(intersection_in_World_xy - projction_of_base_position_in_World) * ...
        cos(elevation_rad) * ...
        moving_direction_vector;

      % % Vector from current base position to desired base position
      % vec_base_cur_to_base_des = ...
      %   norm(intersection_in_World_xy - projction_of_base_position_in_World) * ...
      %   moving_direction_vector;

      desired_base_position = current_base_position_in_World + vec_base_cur_to_base_des;
    end

  end

end
% EOF
