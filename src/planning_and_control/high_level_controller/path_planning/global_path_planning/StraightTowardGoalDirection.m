classdef StraightTowardGoalDirection
% StraightTowardGoalDirection
% Global path planning method that computes a path straight toward the goal from the current robot
% position
%
% Created     : 2024.10.22 by Masazumi Imai
% Last updated: 2024.12.07 by Masazumi Imai

  %% Public Methods
  methods (Access = public)

    function planner = StraightTowardGoalDirection()
    % StraightTowardGoalDirection() Constructor
    end

    function way_points = plan(~, goal_position)
    % plan()
    %   Calculate way points from current position to goal position
      arguments (Input)
        ~;
        goal_position (3, 1) {mustBeA(goal_position, "double")};
      end

      way_points = goal_position;
    end

  end

end  % StraightTowardGoalDirection
