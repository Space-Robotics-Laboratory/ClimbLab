%%%%%% Configuration
%%%%%% config_simulation
%%%%%% 
%%%%%% Configure all simulation parameters
%%%%%% 
%%%%%% Created 2020-07-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-10-15
%%%%%% Keentaro Uno
%
%
% Load configurations for simulation
%
% Function variables:
%
%     OUTPUT
%         robot_param
%         environment_param
%         gait_param
%         control_param
%         equilibrium_eval_param
%         ani_settings
%         save_settings
%         plot_settings
%         gripper_param
%         map_param
%         matching_settings
%     INPUT
%         config

function [robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings] = config_simulation(config)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_param = config_robot_param();
environment_param = config_environment_param();
gait_param = config_gait_param();
control_param = config_control_param();
equilibrium_eval_param = config_equilibrium_param();

ani_settings = config_animation_settings();
save_settings = config_save_settings();
plot_settings = config_plot_settings();
[gripper_param, map_param, matching_settings] = config_target_detection_param();

if strcmp(config,'USER')
    [robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings] = config_USER_param(robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings);
end
if strcmp(config,'gia_static')
    [robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_gia_static(robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'uno_crawl_param')
    [robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_uno_crawl_param(robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'nonperiodic_demo_param')
    [robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings] = config_nonperiodic_demo_param(robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings);
end
if strcmp(config,'iSAIRAS_2020_demo_param')
    [robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings] = config_iSAIRAS_2020_demo_param(robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ...
    ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings);
end

end