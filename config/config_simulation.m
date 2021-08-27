%%%%%% Configuration
%%%%%% config_simulation
%%%%%% 
%%%%%% Configure all simulation parameters
%%%%%% 
%%%%%% Created 2020-07-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-08
%%%%%% Keigo Haji
%
%
% Load configurations for simulation
%
% Function variables:
%
%     OUTPUT
%         robot_param
%         environment_param
%         gait_planning_param
%         control_param
%         equilibrium_eval_param
%         ani_settings
%         save_settings
%         plot_settings
%         gripper_param
%         map_param
%         matching_settings
%         sensing_camera_param
%     INPUT
%         config

function [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_simulation(config)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load all default parameters
[robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_all_default_param();


% Reads the config file with the specified config name and overwrites the variables
if strcmp(config,'USER')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_USER_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param);
end
if strcmp(config,'example_demo_1')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_example_demo_1_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param);
end
if strcmp(config,'example_demo_2')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_example_demo_2_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param);
end
if strcmp(config,'example_demo_3')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_example_demo_3_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param);
end
if strcmp(config,'gia_static')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_gia_static_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'clawar_2021_dynamic_climbing_demo')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_clawar_2021_dynamic_climbing_demo_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'crawl_gait_for_discrete_footholds')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_crawl_gait_for_discrete_footholds_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'nonperiodic_gait_for_discrete_footholds')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_nonperiodic_gait_for_discrete_footholds_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'iSAIRAS_2020_demo')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_iSAIRAS_2020_demo_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param);
end
if strcmp(config,'base_pos_opt_quasistatic')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings] = config_base_pos_opt_quasistatic_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings);
end
if strcmp(config,'clawar_2021_dynamic_climbing_demo')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_clawar_2021_dynamic_climbing_demo_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'clawar_2021_statistical_analysis')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_clawar_2021_statistical_analysis_param(robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'hubrobo_testfield_exp')
    [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_hubrobo_testfield_exp_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param);
end
end