classdef FootholdPlanning
% FootholdPlanning
% Select the next swing limb numbers and foothold positions based on the planner and save these
% history
%
% Created     : 2020.04.13 by Warley Ribeiro
% Last updated: 2024.10.22 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    type (1, 1) string;
    planner;

    swing_limb_id  (:, 1) uint8;
    swing_limb_id_history uint8;

    foothold_positions (3, :) double;
    footholds_history  (:, 1) TrajectoryHistory;

    step_length (1, 1) double;
    step_height (1, 1) double;  % TODO: should be in motion planning?
  end

  %% Public Methods
  methods (Access = public)

    function foothold_planning = FootholdPlanning(config, robot)
    % FootholdPlanning() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigFootholdPlanning")};
        robot  (1, 1) {mustBeA(robot, "Robot")};
      end
      num_limb = robot.LP.getNumberOfLimb();
      current_EE_positions = robot.getEEPosition();

      foothold_planning.type = config.getFootholdSelectionType();
      foothold_planning.planner = foothold_planning.setPlanner();

      foothold_planning.swing_limb_id = uint8(0);
      foothold_planning.swing_limb_id_history = [];

      foothold_planning.foothold_positions = current_EE_positions;
      for limb_id = 1 : num_limb
        foothold_planning.footholds_history(limb_id, 1) = TrajectoryHistory();
        foothold_planning.footholds_history(limb_id, 1) = ...
          foothold_planning.footholds_history(limb_id, 1).addPoint( ...
            current_EE_positions(:, limb_id));
      end

      foothold_planning.step_length = config.getStepLength();
      foothold_planning.step_height = config.getStepHeight();
    end

    function foothold_planning = plan(foothold_planning, ...
        current_time, terrain, path_planning, gait_planning)
    % plan()
    %   Plan the footholds based on the moving direction and graspable points.
      arguments (Input)
        foothold_planning;
        current_time  (1, 1) {mustBeA(current_time,  "double")};
        terrain       (1, 1) {mustBeA(terrain,       "Terrain")};
        path_planning (1, 1) {mustBeA(path_planning, "PathPlanning")};
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      if (foothold_planning.type == "do_nothing")
        return;
      end

      if (~foothold_planning.isUpdateTiming(current_time, gait_planning.getLandingTimings()))
        return;  % Do not update if current time is during motion
      end

      % Update swing limb ID and its history
      foothold_planning.swing_limb_id = foothold_planning.planner.updateSwingLimbNumber( ...
        foothold_planning.swing_limb_id, gait_planning);
      foothold_planning.swing_limb_id_history = ...
        horzcat(foothold_planning.swing_limb_id_history, foothold_planning.swing_limb_id);

      % Update foothold positions
      foothold_planning.foothold_positions = foothold_planning.planner.updateFootholdPositions( ...
        terrain, path_planning, foothold_planning);
      % Update foothold positions history
      num_limb = uint8(size(foothold_planning.foothold_positions, 2));
      for limb_id = 1 : num_limb
        if (any(limb_id ~= foothold_planning.swing_limb_id))
          continue;  % Do not update foothold history for support limb
        end
        foothold_planning.footholds_history(limb_id, 1) = ...
          foothold_planning.footholds_history(limb_id, 1).addPoint( ...
          foothold_planning.foothold_positions(:, limb_id));
      end
    end

  end

  %% Private Methods
  methods (Access = private)

    function boolean = isUpdateTiming(foothold_planning, current_time, landing_time)
      arguments (Input)
        foothold_planning;
        current_time (1, 1) {mustBeA(current_time,  "double")};
        landing_time (1, :) {mustBeA(landing_time,  "double")};
      end
      if (current_time ~= 0.0 && ...
          any(current_time ~= landing_time(1, foothold_planning.swing_limb_id)))
        boolean = false;
      else
        boolean = true;
      end
    end

  end

  %% Setter
  methods (Access = private)

    function planner = setPlanner(foothold_planning)
      switch (foothold_planning.type)
        case "do_nothing"
          planner = [];
        case "fixed_stride"
          planner = FixedStride();
        otherwise
          error("Invalid foothold selection type is specified!!");
      end
    end

  end

  %% Getter
  methods (Access = public)
    function swing_limb_id = getSwingLimbID(foothold_planning)
      swing_limb_id = foothold_planning.swing_limb_id;
    end
    function foothold_positions = getFootholdPositions(foothold_planning, xyz, limb_id)
      arguments (Input)
        foothold_planning;
        xyz (:, 1) uint8 = uint8.empty;
        limb_id (:, 1) uint8 = uint8.empty;
      end
      if (isempty(xyz) && isempty(limb_id))
        xyz = 1 : size(foothold_planning.foothold_positions, 1);
        limb_id = 1 : size(foothold_planning.foothold_positions, 2);
      elseif ((isempty(xyz) || isempty(limb_id)))
        error("ERROR: Need to input both of ""xyz"" and ""limb_id"" " + ...
          "if you want to get componet of ""EE_position"".");
      elseif (any(xyz < 1) || any(xyz > size(foothold_planning.foothold_positions, 1)))
        error("ERROR: First input ""xyz"" must be greater than or equal 1 and " + ...
          "less than or equal 3.");
      elseif (limb_id > size(foothold_planning.foothold_positions, 2) ...
          || limb_id(1, 1) < 1 || limb_id(end, 1) > size(foothold_planning.foothold_positions, 2))
        error("ERROR: Second input ""limb_id"" must be greater than or equal 1 and " + ...
          "less than or equal number of limbs.");
      end
      foothold_positions = foothold_planning.foothold_positions(xyz, limb_id);
    end
    function step_length = getStepLength(foothold_planning)
      step_length = foothold_planning.step_length;
    end

    % TODO: should be in motion planning?
    function step_height = getStepHeight(foothold_planning)
      step_height = foothold_planning.step_height;
    end
  end

end
% EOF
