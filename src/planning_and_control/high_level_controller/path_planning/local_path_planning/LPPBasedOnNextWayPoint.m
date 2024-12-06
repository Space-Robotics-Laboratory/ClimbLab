classdef LPPBasedOnNextWayPoint
% LPPBasedOnNextWayPoint
% Local path planning method that compute the unit vector of the moving direction based on the next
% way point
%
% Created     : 2024.10.22 by Masazumi Imai
% Last updated: 2024.10.22 by Masazumi Imai

  %% Public Methods
  methods (Access = public)

    function planner = LPPBasedOnNextWayPoint()
    % LPPBasedOnNextWayPoint() Constructor
    end

    function moving_direction = plan(~, current_position, global_path)
    % plan()
    %   Calculate the unit vector of the next moving direction
      arguments (Input)
        ~;
        current_position (3, 1) {mustBeA(current_position, "double")};
        global_path (3, :) {mustBeA(global_path, "double")};
      end

      next_way_point = global_path(:, 1);

      move_dir_vec = next_way_point - current_position;
      unit_move_dir_vec = move_dir_vec / norm(move_dir_vec);

      moving_direction = unit_move_dir_vec;
    end

  end

end
% EOF