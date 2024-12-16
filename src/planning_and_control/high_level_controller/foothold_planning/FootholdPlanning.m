classdef FootholdPlanning < handle
% FootholdPlanning
% Select the next swing limb numbers and foothold positions based on the planner and save these
% history
%
% Created     : 2020.04.13 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kType_ (1, 1) string;
    planner_;

    max_allowable_stride_ (1, 1) double;
  end

  %% Public Methods
  methods (Access = public)

    function foothold_planning = FootholdPlanning(config_foothold_planning, robot)
    % FootholdPlanning() Constructor
      arguments (Input)
        config_foothold_planning (1, 1) {mustBeA(config_foothold_planning, "ConfigFootholdPlanning")};
        robot (1, 1) {mustBeA(robot, "Robot")};
      end

      kNumLimb = robot.LP_.getNumberOfLimb();
      current_EE_position = robot.getEEPosition();

      foothold_planning.kType_ = config_foothold_planning.getFootholdSelectionType();
      foothold_planning.setPlanner(kNumLimb);

      foothold_planning.max_allowable_stride_ = config_foothold_planning.getMaxAllowableStride();

      foothold_planning.planner_.output_.setFootholdPosition(current_EE_position);
      foothold_planning.planner_.output_.setFootholdHistory();
    end

    function plan(foothold_planning, current_time, terrain, path_planning, gait_planning)
    % plan()
    %   Plan the footholds based on the moving direction and graspable points.
      arguments (Input)
        foothold_planning;
        current_time  (1, 1) {mustBeA(current_time,  "double")};
        terrain       (1, 1) {mustBeA(terrain,       "Terrain")};
        path_planning (1, 1) {mustBeA(path_planning, "PathPlanning")};
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      if (foothold_planning.kType_ == "do_nothing")
        return;
      end

      if (~foothold_planning.isUpdateTiming(current_time, gait_planning.scheduler_.output_.getLandingTimings()))
        return;  % Do not update if current time is during motion
      end

      % Update swing limb ID and its history
      foothold_planning.planner_.updateSwingLimbId(gait_planning);

      % Update foothold position and its history
      foothold_planning.planner_.updateFootholdPositions(terrain, path_planning, foothold_planning);
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
          any(current_time ~= landing_time(1, foothold_planning.planner_.output_.getSwingLimbId())))
        boolean = false;
      else
        boolean = true;
      end
    end

  end

  %% Setter
  methods (Access = private)

    function setPlanner(foothold_planning, kNumLimb)
      switch (foothold_planning.kType_)
        case "do_nothing"
          planner = [];
        case "fixed_stride"
          planner = FixedStride(kNumLimb);
        otherwise
          error("Invalid foothold selection type is specified!!");
      end
      foothold_planning.planner_ = planner;
    end

  end

  %% Getter
  methods (Access = public)
    function max_allowable_stride = getMaxAllowableStride(foothold_planning)
      max_allowable_stride = foothold_planning.max_allowable_stride_;
    end
  end

end  % FootholdPlanning
