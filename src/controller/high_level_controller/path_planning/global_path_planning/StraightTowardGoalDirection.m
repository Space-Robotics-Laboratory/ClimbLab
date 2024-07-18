classdef StraightTowardGoalDirection < GlobalPathPlanner

  properties (SetAccess = ?GlobalPathPlanner, GetAccess = public)
    moving_direction;  % Unit vector (3x1 double)
  end

  methods (Access = {?PathPlanning, ?GlobalPathPlanner})
    % Constructor
    function global_path = StraightTowardGoalDirection()
      global_path.moving_direction = [0.0; 0.0; 0.0];
    end

    function global_path = plan(global_path, current_position, goal_position)
      % TODO: Input class and use get~ methods to get variables
      % for i = 1:length(varargin)
      %   if class(varargin{i}) == "Robot"
      %     current_position = Robot.getCurrentPosition();
      next_moving_direction = global_path.calcMovingDirection(current_position, goal_position);
      global_path = global_path.setMovingDirection(next_moving_direction);
    end

  end

  methods (Access = private)
    function moving_direction = calcMovingDirection(~, current_position, goal_position)
      move_dir_vec = goal_position - current_position;
      unit_move_dir_vec = move_dir_vec / norm(move_dir_vec);

      moving_direction = unit_move_dir_vec;
    end

    % Setter
    function global_path = setMovingDirection(global_path, next_moving_direction)
      global_path.moving_direction = next_moving_direction;
    end
  end

  methods (Access = public)
    % Getter
    function moving_direction = getMovingDirection(global_path)
      moving_direction = global_path.moving_direction;
    end
  end

end
% EOF