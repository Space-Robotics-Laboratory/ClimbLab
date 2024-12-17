% Main file for climbing/walking simulation
clc; clear; close all;
tic;

%%% Select configuration:
% - "default",
% - "example_demo_1", "example_demo_2", "example_demo_3",
% - "iSAIRAS_2020_demo",
% - "ISTS_2022_",
% - "SII_2022_low_reaction",
% - "ICRA_2023_Reaction_Aware_Motion_Planning",
% - "position_based_impedance_control_for_base",  % UNDER DEVELOPMENT
config = "example_demo_2";

%%% Define a code for current set of simulations
run_cod = config;
% Simulation run identification (date and time)
run_id = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));

global d_time Gravity Ez;

initializeDirectory();

initialize();

figure(animation.getFigure());

for time = 0.0 : d_time : kMaxSimTime
  update();

  if (rem(time, 1 / animation.getFrameRate()) == 0)
    visualizeAnimation();
  end
end

animation.saveVideoFile();


toc
% EOF
