classdef PeriodicGait

  properties (SetAccess = private, GetAccess = public)
    gait_period (1, 1) double;  % [s]
    duty_factor (1, 1) double;

    transfer_duration (1, 1) double;  % [s] including gripper release, limb swing, gripper grasp
    swing_duration    (1, 1) double;  % [s]
    release_duration  (1, 1) double;  % [s]
    grasp_duration    (1, 1) double;  % [s]

    support_duration          (1, 1) double;  % [s]
    all_limb_support_duration (1, 1) double;  % [s]

    sequence uint8;
      % 1st dim: Limb number(s) starting at same timing during gait cycle
      % 2nd dim: Limb number(s) starting at different timing during gait cycle

    swing_timing double;  % [s]
  end
  properties (Access = private)
    % Number of limb motion starting at different timing during gait cycle
    num_limb_motion_starting_at_diff_timing (1, 1) double;
  end

  methods (Access = ?GaitPlanning)
    % Constructor
    function periodic_gait = PeriodicGait()
      periodic_gait.gait_period = 0.0;
      periodic_gait.duty_factor = 0.0;
      periodic_gait.transfer_duration = 0.0;
      periodic_gait.swing_duration = 0.0;
      periodic_gait.release_duration = 0.0;
      periodic_gait.grasp_duration = 0.0;
      periodic_gait.support_duration = 0.0;
      periodic_gait.all_limb_support_duration = 0.0;
      periodic_gait.sequence = [];
      periodic_gait.swing_timing = [];
    end

    function periodic_gait = initialize(periodic_gait, gait_period, duty_factor, ...
      release_duration, grasp_duration, sequence, num_limb)

      periodic_gait.gait_period = gait_period;
      periodic_gait.duty_factor = duty_factor;

      periodic_gait.support_duration = periodic_gait.calcSupportDuration();
      periodic_gait.transfer_duration = periodic_gait.calcTransferDuration();
      periodic_gait.release_duration = release_duration;
      periodic_gait.grasp_duration = grasp_duration;
      periodic_gait.swing_duration = periodic_gait.calcSwingDuration();

      periodic_gait.sequence = sequence;
      periodic_gait.swing_timing = NaN(1, num_limb);

      periodic_gait.num_limb_motion_starting_at_diff_timing = size(periodic_gait.sequence, 2);
      periodic_gait.all_limb_support_duration = periodic_gait.calcAllLimbSupportDuration();
    end

    function periodic_gait = updateSwingTiming(periodic_gait, current_time)
      transfer_duration_ = periodic_gait.transfer_duration;
      all_limb_support_duration_ = periodic_gait.all_limb_support_duration;
      num_limb_motion = periodic_gait.num_limb_motion_starting_at_diff_timing;

      for i = 1:num_limb_motion
        limb_id = periodic_gait.sequence(:, i);

        periodic_gait.swing_timing(1, limb_id) = current_time + ...
          (transfer_duration_ + all_limb_support_duration_) * (i - 1);
      end
    end
  end

  methods (Access = private)
    function support_duration = calcSupportDuration(periodic_gait)
      support_duration = periodic_gait.duty_factor * periodic_gait.gait_period;
    end

    function transfer_duration = calcTransferDuration(periodic_gait)
      transfer_duration = periodic_gait.gait_period - periodic_gait.support_duration;
    end

    function swing_duration = calcSwingDuration(periodic_gait)
      swing_duration = periodic_gait.transfer_duration - ...
        (periodic_gait.release_duration + periodic_gait.grasp_duration);
    end

    function all_limb_support_duration = calcAllLimbSupportDuration(periodic_gait)
      gait_period_ = periodic_gait.gait_period;
      transfer_duration_ = periodic_gait.transfer_duration;
      num_limb_motion = periodic_gait.num_limb_motion_starting_at_diff_timing;
      if (transfer_duration_ * num_limb_motion >= gait_period_)
        all_limb_support_duration = 0.0;
      else
        all_limb_support_duration = ...
          (gait_period_ - transfer_duration_ * num_limb_motion) / num_limb_motion;
      end
    end
  end

  methods (Access = public)
    % Getter
    function swing_duration = getSwingDuration(periodic_gait)
      swing_duration = periodic_gait.swing_duration;
    end
    function swing_timing = getSwingTiming(periodic_gait)
      swing_timing = periodic_gait.swing_timing;
    end

    function visualizeGaitDiagram(periodic_gait, current_time)
      num_limb = size(periodic_gait.swing_timing, 2);
      swing_phase = periodic_gait.swing_timing - current_time;
      region_height = 0.4;

      figure;
      hold on; box on;
      for i = num_limb : -1 : 1
        % Support phase
        yregion(i - region_height/2, i + region_height/2, ...
          FaceColor = [0.0, 0.0, 0.0], FaceAlpha = 1.0, ...
          EdgeColor = [0.0, 0.0, 0.0], EdgeAlpha = 1.0);
        % Swing phase
        rectangle( ...
          "Position", [swing_phase(1, num_limb - i + 1), i - region_height/2, ...
            periodic_gait.transfer_duration, region_height], ...
          FaceColor = [1.0, 1.0, 1.0], EdgeColor = [1.0, 1.0, 1.0]);
      end
      xline(swing_phase, "--", Color = [0.5, 0.5, 0.5], LineWidth = 1.0);
      set(gca, FontName = "Helvetica", FontSize = 25, LineWidth = 1.5, Layer = "top");
      xlim([0, periodic_gait.gait_period]);
      ylim([0.5, num_limb + 0.5]);
      xticks(unique(sort([swing_phase, periodic_gait.gait_period])));
      yticks(1:num_limb);
      yticklabels(flip(["LF", "LH", "RH", "RF"]));
      set(gcf, Color = "w", Position = [100, 100, 700, 400]);
    end
  end

end
% EOF