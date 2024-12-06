classdef PathPlanning
% PathPlanning
% Plan the global and local path from the current robot base position to the goal position
%   Global path: Path to the destination (final goal position)
%   Local path : Path to the way point (next goal position) at current time
%
% Created     : 2021.06.28 by Keigo Haji
% Last updated: 2024.10.22 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    global_path GlobalPathPlanning;
    local_path  LocalPathPlanning;
  end

  %% Public Methods
  methods (Access = public)

    function path_planning = PathPlanning(config, robot, terrain)
    % PathPlanning() Constructor
      arguments (Input)
        config  (1, 1) {mustBeA(config, "ConfigPathPlanning")};
        robot   (1, 1) {mustBeA(robot, "Robot")};
        terrain (1, 1) {mustBeA(terrain, "Terrain")};
      end

      path_planning.global_path = GlobalPathPlanning(config, robot, terrain);
      path_planning.local_path = LocalPathPlanning(config);
    end

    function path_planning = plan(path_planning, robot)
    % plan()
    %   Plan the global and local path
      arguments (Input)
        path_planning;
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      path_planning.global_path = path_planning.global_path.plan();

      path_planning.local_path = path_planning.local_path.plan(robot, path_planning.global_path);
    end

  end

end
% EOF