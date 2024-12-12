classdef GaitSchedulerOutput < handle
% GaitSchedulerOutput
% Gait scheduler output value class
%
% Created     : 2024.12.12 by Masazumi Imai
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    transfer_duration_ (1, 1) double;  % [s] including foot lift up, limb swing, foot lift down durations
    swing_duration_    (1, 1) double;  % [s]

    support_duration_          (1, 1) double;  % [s]
    all_limb_support_duration_ (1, 1) double;  % [s]

    swing_timings_   (1, :) double;  % [s]: Timing of limb transfer motion start for each limb (1xNumLimb)
    landing_timings_ (1, :) double;  % [s]: Timing of limb transfer motion end for each limb (1xNumLimb)
  end

  %% Methods called only from GaitPlanning
  methods (Access = {?PeriodicGait})

    function gait_scheduler_output = GaitSchedulerOutput()
      gait_scheduler_output.transfer_duration_ = 0.0;
      gait_scheduler_output.swing_duration_ = 0.0;
      gait_scheduler_output.support_duration_ = 0.0;
      gait_scheduler_output.all_limb_support_duration_ = 0.0;

      gait_scheduler_output.swing_timings_ = zeros();
      gait_scheduler_output.landing_timings_ = zeros();
    end

  end

  %% Setter
  methods (Access = public)

    function setSupportDuration(gait_scheduler_output, support_duration)
      arguments(Input)
        gait_scheduler_output;
        support_duration (1, 1) {mustBeA(support_duration, "double")};
      end
      gait_scheduler_output.support_duration_ = support_duration;
    end

    function setTransferDuration(gait_scheduler_output, transfer_duration)
      arguments(Input)
        gait_scheduler_output;
        transfer_duration (1, 1) {mustBeA(transfer_duration, "double")};
      end
      gait_scheduler_output.transfer_duration_ = transfer_duration;
    end

    function setSwingDuration(gait_scheduler_output, swing_duration)
      arguments(Input)
        gait_scheduler_output;
        swing_duration (1, 1) {mustBeA(swing_duration, "double")};
      end
      gait_scheduler_output.swing_duration_ = swing_duration;
    end

    function setAllLimbSupportDuration(gait_scheduler_output, all_limb_support_duration)
      arguments(Input)
        gait_scheduler_output;
        all_limb_support_duration (1, 1) {mustBeA(all_limb_support_duration, "double")};
      end
      gait_scheduler_output.all_limb_support_duration_ = all_limb_support_duration;
    end

    function setLimbMotionTimings(gait_scheduler_output, swing_timings, landing_timings)
      arguments(Input)
        gait_scheduler_output;
        swing_timings   (1, :) {mustBeA(swing_timings,   "double")};
        landing_timings (1, :) {mustBeA(landing_timings, "double")};
      end
      gait_scheduler_output.swing_timings_ = swing_timings;
      gait_scheduler_output.landing_timings_ = landing_timings;
    end

  end


  %% Getter
  methods (Access = public)

    function support_duration = getSupportDuration(gait_scheduler_output)
      support_duration = gait_scheduler_output.support_duration_;
    end

    function transfer_duration = getTransferDuration(gait_scheduler_output)
      transfer_duration = gait_scheduler_output.transfer_duration_;
    end

    function all_limb_support_duration = getAllLimbSupportDuration(gait_scheduler_output)
      all_limb_support_duration = gait_scheduler_output.all_limb_support_duration_;
    end

    function swing_timings = getSwingTimings(gait_scheduler_output)
      swing_timings = gait_scheduler_output.swing_timings_;
    end
    function landing_timings = getLandingTimings(gait_scheduler_output)
      landing_timings = gait_scheduler_output.landing_timings_;
    end

  end

end  % GaitSchedulerOutput
