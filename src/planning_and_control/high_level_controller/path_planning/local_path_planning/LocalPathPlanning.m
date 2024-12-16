classdef LocalPathPlanning < handle
% LocalPathPlanning
% Plan the local path from the current robot base position to the goal position
%
% Created     : 2021.06.28 by Keigo Haji
% Last updated: 2024.12.07 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    kType_ (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    planner_;
    moving_direction_ (3, 1) double;  % Unit vector
  end

  %% Public Methods
  methods (Access = public)

    function local_path = LocalPathPlanning(config_path_planning)
    % LocalPathPlanning() Constructor
      arguments (Input)
        config_path_planning (1, 1) {mustBeA(config_path_planning, "ConfigPathPlanning")};
      end

      local_path.kType_ = config_path_planning.getLocalPathPlanningType();
      local_path.planner_ = local_path.setPlanner();
      local_path.moving_direction_ = zeros(3, 1);
    end

    function plan(local_path, robot, global_path)
    % plan()
    %   Plan the local path (next moving direction) to the next waypoint
      arguments (Input)
        local_path;
        robot       (1, 1) {mustBeA(robot,       "Robot")};
        global_path (1, 1) {mustBeA(global_path, "GlobalPathPlanning")};
      end

      if (local_path.kType_ == "do_nothing")
        return;
      end

      current_position = robot.SV_.getBasePosition();
      way_points = global_path.getGlobalPath();
      local_path.moving_direction_ = local_path.planner_.plan(current_position, way_points);
    end

  end

  %% Setter
  methods (Access = private)

    function planner = setPlanner(local_path)
      switch (local_path.kType_)
        case "do_nothing"
          planner = [];
        case "LPP_based_on_next_way_point"
          planner = LPPBasedOnNextWayPoint();
        otherwise
          error("ERROR: Invalid local path planner is specified!");
      end
    end

  end

  %% Getter
  methods (Access = public)
    function moving_direction = getMovingDirection(local_path)
      moving_direction = local_path.moving_direction_;
    end
  end

end  % LocalPathPlanning
