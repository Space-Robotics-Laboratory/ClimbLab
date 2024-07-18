classdef GaitPlanning

  properties (SetAccess = immutable, GetAccess = public)
    type (1, 1) string;
  end
  properties (SetAccess = private, GetAccess = public)
    scheduler;
  end

  methods (Access = public)
    % Constructor
    function gait_planning = GaitPlanning(gait_type)
      gait_planning.type = gait_type;

      if startsWith(gait_planning.type, "periodic")
        gait_planning.scheduler = PeriodicGait();
      elseif startsWith(gait_planning.type, "non_periodic")
        gait_planning.scheduler = NonPeriodicGait();
      end
    end

    function gait_planning = initializeGait(gait_planning, ...
        gait_period, duty_factor, release_duration, grasp_duration, sequence, num_limb)
      gait_planning.scheduler = gait_planning.scheduler.initialize( ...
        gait_period, duty_factor, release_duration, grasp_duration, sequence, num_limb);
    end

    function gait_planning = update(gait_planning, current_time)
      gait_planning.scheduler = gait_planning.scheduler.updateSwingTiming(current_time);
    end

  end

end
% EOF