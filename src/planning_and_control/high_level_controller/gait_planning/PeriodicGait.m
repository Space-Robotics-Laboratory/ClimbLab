classdef PeriodicGait < handle
% PeriodicGait
% Periodic gait scheduler
%
% Created     : 2021.04.20 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kGaitPeriod_ (1, 1) double;  % [s]
    kDutyFactor_ (1, 1) double;
    % Gait sequence
    % 1st dim: Limb number(s) starting at same timing during gait cycle
    % 2nd dim: Limb number(s) starting at different timing during gait cycle
    kSequence_ uint8;

    output_ GaitSchedulerOutput;
  end
  properties (Access = public)
    % Number of limb motions starting at different timing during gait cycle
    kNumLimbMotionStartingAtDiffTiming (1, 1) double;
  end

  %% Methods called only from GaitPlanning
  methods (Access = ?GaitPlanning)

    function periodic_gait = PeriodicGait(config_gait_planning)
    % PeriodicGait() Constructor
      arguments (Input)
        config_gait_planning (1, 1) {mustBeA(config_gait_planning, "ConfigGaitPlanning")};
      end
      periodic_gait.kGaitPeriod_ = config_gait_planning.getGaitPeriod();
      periodic_gait.kDutyFactor_ = config_gait_planning.getDutyFactor();
      periodic_gait.kSequence_ = config_gait_planning.getGaitSequence();
      periodic_gait.kNumLimbMotionStartingAtDiffTiming = size(periodic_gait.kSequence_, 2);

      periodic_gait.output_ = GaitSchedulerOutput();
    end

    function calcSupportDuration(periodic_gait)
      support_duration = periodic_gait.kDutyFactor_ * periodic_gait.kGaitPeriod_;
      periodic_gait.output_.setSupportDuration(support_duration);
    end

    function calcTransferDuration(periodic_gait)
      support_duration = periodic_gait.output_.getSupportDuration();
      transfer_duration = periodic_gait.kGaitPeriod_ - support_duration;
      periodic_gait.output_.setTransferDuration(transfer_duration);
    end

    function calcSwingDuration(periodic_gait, foot_lift_up_duration, foot_lift_down_duration)
      arguments (Input)
        periodic_gait;
        foot_lift_up_duration   (1, 1) {mustBeA(foot_lift_up_duration, "double")};
        foot_lift_down_duration (1, 1) {mustBeA(foot_lift_down_duration, "double")};
      end

      transfer_duration = periodic_gait.output_.getTransferDuration();

      swing_duration = transfer_duration - (foot_lift_up_duration + foot_lift_down_duration);
      periodic_gait.output_.setSwingDuration(swing_duration);
    end

    function calcAllLimbSupportDuration(periodic_gait)
      gait_period = periodic_gait.kGaitPeriod_;
      transfer_duration = periodic_gait.output_.getTransferDuration();
      kNumLimbMotion = periodic_gait.kNumLimbMotionStartingAtDiffTiming;

      if (transfer_duration * kNumLimbMotion >= gait_period)
        all_limb_support_duration = 0.0;
      else
        all_limb_support_duration = ...
          (gait_period - transfer_duration * kNumLimbMotion) / kNumLimbMotion;
      end

      periodic_gait.output_.setAllLimbSupportDuration(all_limb_support_duration);
    end

    function initializeLimbMotionTimings(periodic_gait)
      transfer_duration = periodic_gait.output_.getTransferDuration();
      all_limb_support_duration = periodic_gait.output_.getAllLimbSupportDuration();
      kNumLimbMotion = periodic_gait.kNumLimbMotionStartingAtDiffTiming;
      kNumLimb = numel(periodic_gait.kSequence_);

      swing_timings = zeros(1, kNumLimb);
      for i = 1 : kNumLimbMotion
        limb_id = periodic_gait.kSequence_(:, i);

        swing_timings(1, limb_id) = 0.0 + ...
          (transfer_duration + all_limb_support_duration) * (i - 1);
      end

      landing_timings = swing_timings + transfer_duration;

      periodic_gait.output_.setLimbMotionTimings(swing_timings, landing_timings);
    end

    function updateSwingAndLandingTiming(periodic_gait, current_time, swing_limb_id)
      arguments (Input)
        periodic_gait;
        current_time  (1, 1) {mustBeA(current_time, "double")};
        swing_limb_id (1, 1) {mustBeA(swing_limb_id, "uint8")};
      end

      [~, idx] = find(periodic_gait.kSequence_ == swing_limb_id(1, 1));
      sequence_tmp = [periodic_gait.kSequence_(:, idx:end), periodic_gait.kSequence_(:, 1:idx-1)];

      transfer_duration = periodic_gait.output_.getTransferDuration();
      all_limb_support_duration = periodic_gait.output_.getAllLimbSupportDuration();
      kNumLimbMotion = periodic_gait.kNumLimbMotionStartingAtDiffTiming;
      swing_timings = periodic_gait.output_.getSwingTimings();

      for i = 1 : kNumLimbMotion
        limb_id = sequence_tmp(:, i);
        swing_timings(1, limb_id) = current_time + ...
          (transfer_duration + all_limb_support_duration) * (i - 1);
      end

      landing_timings = swing_timings + transfer_duration;

      periodic_gait.output_.setLimbMotionTimings(swing_timings, landing_timings);
    end

  end

  %% Getter
  methods (Access = public)
    function sequence = getSequence(periodic_gait)
      sequence = periodic_gait.kSequence_;
    end
  end

  %% Methods for Visualization
  methods (Access = public)

    % function visualizeGaitDiagram(periodic_gait, current_time)
    %   num_limb = size(periodic_gait.swing_timing, 2);
    %   swing_phase = periodic_gait.swing_timing - current_time;
    %   region_height = 0.4;

    %   figure;
    %   hold on; box on;
    %   for i = num_limb : -1 : 1
    %     % Support phase
    %     yregion(i - region_height/2, i + region_height/2, ...
    %       FaceColor = [0.0, 0.0, 0.0], FaceAlpha = 1.0, ...
    %       EdgeColor = [0.0, 0.0, 0.0], EdgeAlpha = 1.0);
    %     % Swing phase
    %     rectangle( ...
    %       "Position", [swing_phase(1, num_limb - i + 1), i - region_height/2, ...
    %         periodic_gait.transfer_duration, region_height], ...
    %       FaceColor = [1.0, 1.0, 1.0], EdgeColor = [1.0, 1.0, 1.0]);
    %   end
    %   xline(swing_phase, "--", Color = [0.5, 0.5, 0.5], LineWidth = 1.0);
    %   set(gca, FontName = "Helvetica", FontSize = 25, LineWidth = 1.5, Layer = "top");
    %   xlim([0, periodic_gait.kGaitPeriod_]);
    %   ylim([0.5, num_limb + 0.5]);
    %   xticks(unique(sort([swing_phase, periodic_gait.kGaitPeriod_])));
    %   yticks(1:num_limb);
    %   yticklabels(flip(["LF", "LH", "RH", "RF"]));
    %   set(gcf, Color = "w", Position = [100, 100, 700, 400]);
    % end

  end

end  % PeriodicGait
