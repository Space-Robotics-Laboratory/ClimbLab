classdef PathPlanning < handle
% PathPlanning
% Plan the global and local path from the current robot base position to the goal position
%   Global path: Path to the destination (final goal position)
%   Local path : Path to the way point (next goal position) at current time
%
% Created     : 2021.06.28 by Keigo Haji
% Last updated: 2024.12.07 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    global_path_ GlobalPathPlanning;
    local_path_  LocalPathPlanning;
  end

  %% Public Methods
  methods (Access = public)

    function path_planning = PathPlanning(config_path_planning, robot, terrain)
    % PathPlanning() Constructor
      arguments (Input)
        config_path_planning  (1, 1) {mustBeA(config_path_planning, "ConfigPathPlanning")};
        robot   (1, 1) {mustBeA(robot, "Robot")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end

      path_planning.global_path_ = GlobalPathPlanning(config_path_planning, robot, terrain);
      path_planning.local_path_ = LocalPathPlanning(config_path_planning);
    end

    function plan(path_planning, robot)
    % plan()
    %   Plan the global and local path
      arguments (Input)
        path_planning;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      path_planning.global_path_.plan();

      path_planning.local_path_.plan(robot, path_planning.global_path_);
    end

  end

  %% Getter
  methods (Access = public)

    function global_path = getGlobalPath(path_planning)
      global_path = path_planning.global_path_;
    end

    function local_path = getLocalPath(path_planning)
      local_path = path_planning.local_path_;
    end

  end

end  % PathPlanning
