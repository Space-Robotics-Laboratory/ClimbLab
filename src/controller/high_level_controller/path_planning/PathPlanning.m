classdef PathPlanning

  properties (SetAccess = immutable, GetAccess = public)
    global_path_plan_type (1, 1) string;
    local_path_plan_type (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    global_path;
    local_path;
  end

  methods (Access = public)
    % Constructor
    function path_planning = PathPlanning(global_type, local_type)
      validateattributes(global_type, "string", {"size", [1, 1]});
      validateattributes(local_type, "string", {"size", [1, 1]});

      path_planning.global_path_plan_type = global_type;
      path_planning.local_path_plan_type = local_type;

      path_planning.global_path = path_planning.setGlobalPathPlanner(path_planning.global_path_plan_type);
      path_planning.local_path = path_planning.setLoalPathPlanner(path_planning.local_path_plan_type);
    end

    function path_planning = planGlobalPath(path_planning, current_position, goal_position)
      path_planning.global_path = path_planning.global_path.plan(current_position, goal_position);
    end
  end

  methods (Access = private)

    function global_path = setGlobalPathPlanner(~, global_path_plan_type)
      switch global_path_plan_type
        case "straight_toward_the_goal_directory"
          global_path = StraightTowardGoalDirection();
        otherwise
          error("Invalid global path planner is specified!!");
      end
    end

    function local_path = setLoalPathPlanner(~, local_path_plan_type)
      switch local_path_plan_type
        case ""
          local_path = [];
        otherwise
          error("Invalid local path planner is specified!!");
      end
    end

  end

end
% EOF