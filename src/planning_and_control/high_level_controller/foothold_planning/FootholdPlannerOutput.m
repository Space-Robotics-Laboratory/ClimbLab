classdef FootholdPlannerOutput < handle
% FixedStride
% Foothold planning method. Select the next swing limb numbers based on the periodic gait sequence
% and foothold positions based on the moving direction and graspable points.
%
% Created     : 2024.12.12 by Masazumi Imai
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    swing_limb_id_  (:, 1) uint8;
    swing_limb_id_history_ uint8;

    foothold_position_ (3, :) double;
    foothold_history_  (:, 1) TrajectoryHistory;
  end

  %% Public Methods
  methods (Access = public)

    function foothold_planner_output = FootholdPlannerOutput(kNumLimb)
    % FootholdPlannerOutput() Constructor
      arguments (Input)
        kNumLimb (1, 1) {mustBeA(kNumLimb, "uint8")};
      end

      foothold_planner_output.swing_limb_id_ = uint8(0);
      foothold_planner_output.swing_limb_id_history_ = [];
      foothold_planner_output.foothold_position_ = zeros(3, kNumLimb);
      for limb_id = 1 : kNumLimb
        foothold_planner_output.foothold_history_(limb_id, 1) = TrajectoryHistory();
      end
    end

  end

  %% Setter
  methods (Access = public)

    function setSwingLimbId(foothold_planner_output, next_swing_limb_id)
      arguments (Input)
        foothold_planner_output;
        next_swing_limb_id (1, 1) {mustBeA(next_swing_limb_id, "uint8")};
      end

      foothold_planner_output.swing_limb_id_ = next_swing_limb_id;
    end

    function setSwingLimbIdHistory(foothold_planner_output, next_swing_limb_id)
      arguments (Input)
        foothold_planner_output;
        next_swing_limb_id (1, 1) {mustBeA(next_swing_limb_id, "uint8")};
      end

      foothold_planner_output.swing_limb_id_history_ = ...
        horzcat(foothold_planner_output.swing_limb_id_history_, next_swing_limb_id);
    end

    function setFootholdPosition(foothold_planner_output, next_foothold_position)
      arguments (Input)
        foothold_planner_output;
        next_foothold_position (3, :) {mustBeA(next_foothold_position, "double")};
      end

      foothold_planner_output.foothold_position_ = next_foothold_position;
    end

    function setFootholdHistory(foothold_planner_output, limb_id_array)
      arguments (Input)
        foothold_planner_output;
        limb_id_array (:, 1) uint8 = uint8.empty;
      end

      if (isempty(limb_id_array))
        limb_id_array = 1 : size(foothold_planner_output.foothold_position_, 2);
      % elseif (limb_id)
      end

      kNumLimb = size(foothold_planner_output.foothold_position_, 2);

      for limb_id = 1 : kNumLimb
        if (~any(limb_id == limb_id_array))
          continue;
        end

        foothold_planner_output.foothold_history_(limb_id, 1).addPoint( ...
          foothold_planner_output.foothold_position_(:, limb_id));
      end
    end

  end

  %% Getter
  methods (Access = public)
    function swing_limb_id = getSwingLimbId(foothold_planner_output)
      swing_limb_id = foothold_planner_output.swing_limb_id_;
    end

    function foothold_positions = getFootholdPosition(foothold_planner_output, xyz, limb_id)
      arguments (Input)
        foothold_planner_output;
        xyz (:, 1) uint8 = uint8.empty;
        limb_id (:, 1) uint8 = uint8.empty;
      end

      if (isempty(xyz) && isempty(limb_id))
        xyz = 1 : size(foothold_planner_output.foothold_position_, 1);
        limb_id = 1 : size(foothold_planner_output.foothold_position_, 2);
      elseif ((isempty(xyz) || isempty(limb_id)))
        error("ERROR: Need to input both of ""xyz"" and ""limb_id"" " + ...
          "if you want to get component of ""EE_position"".");
      elseif (any(xyz < 1) || any(xyz > size(foothold_planner_output.foothold_position_, 1)))
        error("ERROR: First input ""xyz"" must be greater than or equal 1 and " + ...
          "less than or equal 3.");
      elseif (limb_id > size(foothold_planner_output.foothold_position_, 2) ...
          || limb_id(1, 1) < 1 || limb_id(end, 1) > size(foothold_planner_output.foothold_position_, 2))
        error("ERROR: Second input ""limb_id"" must be greater than or equal 1 and " + ...
          "less than or equal number of limbs.");
      end

      foothold_positions = foothold_planner_output.foothold_position_(xyz, limb_id);
    end
  end

end  % FootholdPlannerOutput
