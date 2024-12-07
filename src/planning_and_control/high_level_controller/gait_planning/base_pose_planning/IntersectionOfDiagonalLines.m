classdef IntersectionOfDiagonalLines
% IntersectionOfDiagonalLines
% Calculate the robot base position based on the intersection of diagonal lines formed by diagonal foothold positions
%
% Created     : 2024.09.29 by Masazumi Imai
% Last updated: 2024.12.07 by Masazumi Imai

  %% Public Methods
  methods (Access = public)

    function planner = IntersectionOfDiagonalLines()
    end

    function desired_base_position_in_World = plan(planner, varargin)
    % plan()
    %   Plan the desired base position in World frame
    %   Input - robot Robot
    %         - foothold_planning FootholdPlanning
      for input_id = 1 : length(varargin)
        switch (class(varargin{input_id}))
          case "Robot"
            robot = varargin{input_id};
          case "FootholdPlanning"
            foothold_planning = varargin{input_id};
          otherwise
            continue;
        end
      end
      if (~exist("robot", "var") || ~exist("foothold_planning", "var"))
        error("ERROR: Input is not correct.");
      end

      current_base_position_in_World = robot.SV.getBasePosition();
      current_base_orientation_in_World = robot.SV.getBaseOrientationDCM();

      kNumLimb = robot.LP.getNumberOfLimb();

      current_EE_positions_in_World = robot.getEEPosition();
      desired_EE_positions_in_World = foothold_planning.getFootholdPositions();

      current_EE_positions_in_Base = zeros(3, kNumLimb);
      desired_EE_positions_in_Base = zeros(3, kNumLimb);
      for limb_id = 1 : kNumLimb
        current_EE_positions_in_Base(:, limb_id) = current_base_orientation_in_World' * ...
          (current_EE_positions_in_World(:, limb_id) - current_base_position_in_World);
        desired_EE_positions_in_Base(:, limb_id) = current_base_orientation_in_World' * ...
          (desired_EE_positions_in_World(:, limb_id) - current_base_position_in_World);
      end

      current_EE_positions_in_Base_xy = [current_EE_positions_in_Base(1:2, :); zeros(1, kNumLimb)];
      desired_EE_positions_in_Base_xy = [desired_EE_positions_in_Base(1:2, :); zeros(1, kNumLimb)];

      swing_limb_id = foothold_planning.getSwingLimbID();

      [diagonal_lines, comb_diag_limb] = planner.calcDiagonalLines( ...
        current_EE_positions_in_Base_xy, desired_EE_positions_in_Base_xy, kNumLimb, swing_limb_id);

      intersection_in_Base_xy = planner.calcIntersectionInBaseXYPlane( ...
        diagonal_lines, comb_diag_limb);

      desired_base_position_in_World = current_base_position_in_World + ...
        current_base_orientation_in_World * intersection_in_Base_xy;
    end

  end

  %% Private Methods
  methods (Access = private)

    function [diagonal_lines, comb_diag_limb] = calcDiagonalLines(~, ...
        current_EE_positions_in_Base_xy, desired_EE_positions_in_Base_xy, kNumLimb, swing_limb_id)
    % calcDiagonalLines()
    %   Calculate diagonal lines formed by non-adjacent limbs
      arguments (Input)
        ~;
        current_EE_positions_in_Base_xy ...
                      (3, :) {mustBeA(current_EE_positions_in_Base_xy, "double")};
        desired_EE_positions_in_Base_xy ...
                      (3, :) {mustBeA(desired_EE_positions_in_Base_xy, "double")};
        kNumLimb      (1, 1) {mustBeA(kNumLimb, "uint8")};
        swing_limb_id (1, 1) {mustBeA(swing_limb_id, "uint8")};
      end

      % EE2EE_line (3, 2, i, j): Start and end positions of line connecting i-th and j-th EE
      %   EE2EE_line(:, 1, i, j): Position of i-th EE to form a line with j-th EE
      %   EE2EE_line(:, 2, i, j): Position of j-th EE to form a line with i-th EE
      EE2EE_line = zeros(3, 2, kNumLimb, kNumLimb);
      % Combination of 2 limbs selected from all limbs
      comb_limbs = nchoosek(1:kNumLimb, 2);
      % Combination of limbs forming a diagonal line
      %   1st dim: Number of combinations
      %   2nd dim: Limb ID to form a diagonal line
      comb_diag_limb = uint8.empty;
      % Count the number of limb combinations that form a diagonal line
      cnt = 1;

      % diagonal_lines(3, 2, i, j): Start and end positions of line connecting i-th and j-th EE
      %   diagonal_lines(:, 1, i, j): Position of i-th EE to form a diagonal line with j-th EE
      %   diagonal_lines(:, 2, i, j): Position of j-th EE to form a diagonal line with i-th EE
      diagonal_lines = zeros(3, 2, kNumLimb, kNumLimb);

      % Calculation of lines with limbs that form combinations
      for k = 1 : size(comb_limbs, 1)
        i = comb_limbs(k, 1);
        j = comb_limbs(k, 2);
        % Limb ID before and after i (i-1, i+1)
        if (i == 1)
          i_minus_1 = kNumLimb;
        else
          i_minus_1 = i - 1;
        end
        i_plus_1 = i + 1;
        % Limb ID before and after j (j-1, j+1)
        if (j == 1)
          j_minus_1 = kNumLimb;
        else
          j_minus_1 = j - 1;
        end
        if (j == kNumLimb)
          j_plus_1 = 1;
        else
          j_plus_1 = j + 1;
        end

        % If i is swing limb, EE2EE_line(:, 1, i, j) is next position of swing limb
        if (i == swing_limb_id)
          EE2EE_line(:, 1, i, j) = desired_EE_positions_in_Base_xy(:, i);
          EE2EE_line(:, 2, i, j) = current_EE_positions_in_Base_xy(:, j);
          % Diagonal line by the i-th limb
          if (j ~= i_minus_1 && j ~= i_plus_1)
            diagonal_lines(:, 1, i, j) = desired_EE_positions_in_Base_xy(:, i);
            diagonal_lines(:, 2, i, j) = current_EE_positions_in_Base_xy(:, j);
            comb_diag_limb(cnt, :) = [i, j];
            cnt = cnt + 1;
          end
        % If j is swing limb, EE2EE_line(:, 2, i, j) is next position of swing limb
        elseif (j == swing_limb_id)
          EE2EE_line(:, 1, i, j) = current_EE_positions_in_Base_xy(:, i);
          EE2EE_line(:, 2, i, j) = desired_EE_positions_in_Base_xy(:, j);
          % Diagonal line by the j-th limb
          if (i ~= j_minus_1 && i ~= j_plus_1)
            diagonal_lines(:, 1, i, j) = current_EE_positions_in_Base_xy(:, i);
            diagonal_lines(:, 2, i, j) = desired_EE_positions_in_Base_xy(:, j);
            comb_diag_limb(cnt, :) = [i, j];
            cnt = cnt + 1;
          end
        % If i and j are supporting limb
        else
          EE2EE_line(:, 1, i, j) = current_EE_positions_in_Base_xy(:, i);
          EE2EE_line(:, 2, i, j) = current_EE_positions_in_Base_xy(:, j);
          % Diagonal line by the i-th limb
          if (j ~= i_minus_1 && j ~= i_plus_1)
            diagonal_lines(:, 1, i, j) = current_EE_positions_in_Base_xy(:, i);
            diagonal_lines(:, 2, i, j) = current_EE_positions_in_Base_xy(:, j);
            comb_diag_limb(cnt, :) = [i, j];
            cnt = cnt + 1;
          end
        end
      end
    end

    function intersection_in_Base_xy = calcIntersectionInBaseXYPlane(~, ...
        diagonal_lines, comb_diag_limb)
    % calcIntersectionInBaseXYPlane()
    %   Calculate the intersection of diagonal lines in x-y plane of Base frame
    %   NOTE: This function is currently written for 4-limbed robot
      arguments (Input)
        ~;
        diagonal_lines (3, 2, :, :) {mustBeA(diagonal_lines, "double")};
        comb_diag_limb (:, 2) {mustBeA(comb_diag_limb, "uint8")};
      end

      %%% Vectors needed to calculate intersection
      vec_diag_line_1 = diagonal_lines(:, 2, comb_diag_limb(1, 1), comb_diag_limb(1, 2)) - ...
        diagonal_lines(:, 1, comb_diag_limb(1, 1), comb_diag_limb(1, 2));
      vec_diag_line_2 = diagonal_lines(:, 2, comb_diag_limb(2, 1), comb_diag_limb(2, 2)) - ...
        diagonal_lines(:, 1, comb_diag_limb(2, 1), comb_diag_limb(2, 2));
      % Vector from stat point of "vec_diag_line_1" to stat point of "vec_diag_line_2"
      vec_1s_to_2s = diagonal_lines(:, 1, comb_diag_limb(1, 1), comb_diag_limb(1, 2)) - ...
        diagonal_lines(:, 1, comb_diag_limb(2, 1), comb_diag_limb(2, 2));

      % Ratio from start point of "vec_diag_line_1" to the intersection
      ratio = - cross(vec_diag_line_2, vec_diag_line_1) \ cross(vec_diag_line_2, vec_1s_to_2s);

      % Intersection point in x-y plane of Base frame
      intersection_in_Base_xy = ...
        diagonal_lines(:, 1, comb_diag_limb(1, 1), comb_diag_limb(1, 2)) + vec_diag_line_1 * ratio;
    end

  end

end  % IntersectionOfDiagonalLines
