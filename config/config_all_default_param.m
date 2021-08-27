%%%%%% Configuration
%%%%%% config_all_default_param
%%%%%% 
%%%%%% Define all default parameters
%%%%%% 
%%%%%% Created 2021-03-02
%%%%%% Keigo Haji
%%%%%% Last update: 2021-04-24
%%%%%% Kentaro Uno
%
%
% Define and load configurations for all default parameters. 
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
%         -
%
%
function [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_all_default_param()


% Load all default parameters
robot_param = config_robot_param();
environment_param = config_environment_param();
gait_planning_param = config_gait_planning_param();
control_param = config_control_param();
equilibrium_eval_param = config_equilibrium_param();
ani_settings = config_animation_settings();
save_settings = config_save_settings(environment_param);
plot_settings = config_plot_settings();
[gripper_param, map_param, matching_settings] = config_target_detection_param();
sensing_camera_param = config_sensing_camera_param();


end

