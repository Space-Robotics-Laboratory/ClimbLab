classdef GaitPlanningTest < matlab.unittest.TestCase
  % gait_planning_test = GaitPlanningTest;
  % result = gait_planning_test.run

  methods (Test)
    function testPeriodicCrawl(periodic_crawl_gait_test)
      gait_type = "periodic_crawl";
      release_duration = 0.0;
      grasp_duration = 0.0;
      num_limb = 4;
      gait_period = 4.0;
      duty_factor = 0.75;
      sequence = [2, 1, 3, 4];
      time = 0.0;

      gait_planning = GaitPlanning(gait_type);

      gait_planning = gait_planning.initializeGait(gait_period, duty_factor, ...
        release_duration, grasp_duration, sequence, num_limb);

      gait_planning = gait_planning.update(time);

      swing_timing = gait_planning.scheduler.getSwingTiming();
      expected_swing_timing = [1.0, 0.0, 2.0, 3.0];
      periodic_crawl_gait_test.verifyEqual(swing_timing, expected_swing_timing);

      gait_planning.scheduler.visualizeGaitDiagram(time);
    end

    function testPeriodicTrot(periodic_trot_gait_test)
      gait_type = "periodic_trot";
      release_duration = 0.0;
      grasp_duration = 0.0;
      num_limb = 4;
      gait_period = 1.0;
      duty_factor = 0.5;
      sequence = [1, 2; 3, 4];
      time = 0.0;

      gait_planning = GaitPlanning(gait_type);

      gait_planning = gait_planning.initializeGait(gait_period, duty_factor, ...
        release_duration, grasp_duration, sequence, num_limb);

      gait_planning = gait_planning.update(time);

      swing_timing = gait_planning.scheduler.getSwingTiming();
      expected_swing_timing = [0.0, 0.5, 0.0, 0.5];
      periodic_trot_gait_test.verifyEqual(swing_timing, expected_swing_timing);

      gait_planning.scheduler.visualizeGaitDiagram(time);
    end

  end

end
% EOF