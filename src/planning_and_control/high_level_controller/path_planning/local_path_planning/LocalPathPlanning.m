classdef LocalPathPlanning
% LocalPathPlanning
% Plan the local path from the current robot base position to the goal position
%
% Created     : 2021.06.28 by Keigo Haji
% Last updated: 2024.10.22 by Masazumi Imai

  %% Properties
  properties (SetAccess = immutable, GetAccess = public)
    type (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    planner;
    moving_direction (3, 1) double;  % Unit vector
  end

  %% Public Methods
  methods (Access = public)

    function local_path = LocalPathPlanning(config)
    % LocalPathPlanning() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigPathPlanning")};
      end
      local_path.type = config.getLocalPathPlanningType();
      local_path.planner = local_path.setPlanner();
      local_path.moving_direction = zeros(3, 1);
    end

    function local_path = plan(local_path, robot, global_path)
    % plan()
    %   Plan the local path (next moving direction) to the next waypoint
      arguments (Input)
        local_path;
        robot (1, 1) {mustBeA(robot, "Robot")};
        global_path (1, 1) {mustBeA(global_path, "GlobalPathPlanning")};
      end
      if (local_path.type == "do_nothing")
        return;
      end

      current_position = robot.SV.getBasePosition();
      way_points = global_path.getGlobalPath();
      local_path.moving_direction = local_path.planner.plan(current_position, way_points);
    end

  end

  %% Setter
  methods (Access = private)

    function planner = setPlanner(local_path)
      switch (local_path.type)
        case "do_nothing"
          planner = [];
        case "LPP_based_on_next_way_point"
          planner = LPPBasedOnNextWayPoint();
        otherwise
          error("ERROR: Invalid local path planner is specified!!");
      end
    end

  end

  %% Getter
  methods (Access = public)
    function moving_direction = getMovingDirection(local_path)
      moving_direction = local_path.moving_direction;
    end
  end

end
% EOF