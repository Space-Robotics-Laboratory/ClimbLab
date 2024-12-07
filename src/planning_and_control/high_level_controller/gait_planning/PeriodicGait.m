classdef PeriodicGait
% PeriodicGait
% Periodic gait scheduler
%
% Created     : 2021.04.20 by Warley Ribeiro
% Last updated: 2024.12.07 by Masazumi Imai

  %% Properties
  properties (SetAccess = private, GetAccess = public)
    kGaitPeriod_ (1, 1) double;  % [s]
    kDutyFactor_ (1, 1) double;
    % Gait sequence
    % 1st dim: Limb number(s) starting at same timing during gait cycle
    % 2nd dim: Limb number(s) starting at different timing during gait cycle
    kSequence_ uint8;
  end
  properties (Access = public)
    % Number of limb motions starting at different timing during gait cycle
    kNumLimbMotionStartingAtDiffTiming (1, 1) double;
  end

  %% Methods called only from GaitPlanning
  methods (Access = ?GaitPlanning)

    function periodic_gait = PeriodicGait(config)
    % PeriodicGait() Constructor
      arguments (Input)
        config (1, 1) {mustBeA(config, "ConfigGaitPlanning")};
      end
      periodic_gait.kGaitPeriod_ = config.getGaitPeriod();
      periodic_gait.kDutyFactor_ = config.getDutyFactor();
      periodic_gait.kSequence_ = config.getGaitSequence();
      periodic_gait.kNumLimbMotionStartingAtDiffTiming = size(periodic_gait.kSequence_, 2);
    end

    function support_duration = calcSupportDuration(periodic_gait)
      support_duration = periodic_gait.kDutyFactor_ * periodic_gait.kGaitPeriod_;
    end

    function transfer_duration = calcTransferDuration(periodic_gait, support_duration)
      arguments (Input)
        periodic_gait;
        support_duration (1, 1) {mustBeA(support_duration, "double")};
      end
      transfer_duration = periodic_gait.kGaitPeriod_ - support_duration;
    end

    function swing_duration = calcSwingDuration(~, gait_planning)
      arguments (Input)
        ~;
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end
      swing_duration = gait_planning.getTransferDuration() - ...
        (gait_planning.getFootLiftUpDuration() + gait_planning.getFootLiftDownDuration());
    end

    function all_limb_support_duration = calcAllLimbSupportDuration(periodic_gait, gait_planning)
      arguments (Input)
        periodic_gait;
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      gait_period = periodic_gait.kGaitPeriod_;
      transfer_duration = gait_planning.getTransferDuration();
      num_limb_motion = periodic_gait.kNumLimbMotionStartingAtDiffTiming;

      if (transfer_duration * num_limb_motion >= gait_period)
        all_limb_support_duration = 0.0;
      else
        all_limb_support_duration = ...
          (gait_period - transfer_duration * num_limb_motion) / num_limb_motion;
      end
    end

    function [swing_timings, landing_timings] = initializeLimbMotionTimings(periodic_gait, ...
        gait_planning)
      arguments (Input)
        periodic_gait;
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
      end

      transfer_duration = gait_planning.getTransferDuration();
      all_limb_support_duration = gait_planning.getAllLimbSupportDuration();
      num_limb_motion = periodic_gait.kNumLimbMotionStartingAtDiffTiming;
      num_limb = numel(periodic_gait.kSequence_);

      swing_timings = zeros(1, num_limb);
      for i = 1 : num_limb_motion
        limb_id = periodic_gait.kSequence_(:, i);

        swing_timings(1, limb_id) = 0.0 + ...
          (transfer_duration + all_limb_support_duration) * (i - 1);
      end

      landing_timings = swing_timings + transfer_duration;
    end

    function [swing_timings, landing_timings] = updateSwingAndLandingTiming(periodic_gait, ...
        current_time, gait_planning, swing_limb_id)
      arguments (Input)
        periodic_gait;
        current_time  (1, 1) {mustBeA(current_time, "double")};
        gait_planning (1, 1) {mustBeA(gait_planning, "GaitPlanning")};
        swing_limb_id (1, 1) {mustBeA(swing_limb_id, "uint8")};
      end
      [~, idx] = find(periodic_gait.kSequence_ == swing_limb_id(1, 1));
      sequence_tmp = [periodic_gait.kSequence_(:, idx:end), periodic_gait.kSequence_(:, 1:idx-1)];

      transfer_duration = gait_planning.getTransferDuration();
      all_limb_support_duration = gait_planning.getAllLimbSupportDuration();
      num_limb_motion = periodic_gait.kNumLimbMotionStartingAtDiffTiming;
      swing_timings = gait_planning.getSwingTimings();

      for i = 1 : num_limb_motion
        limb_id = sequence_tmp(:, i);
        swing_timings(1, limb_id) = current_time + ...
          (transfer_duration + all_limb_support_duration) * (i - 1);
      end

      landing_timings = swing_timings + transfer_duration;
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
