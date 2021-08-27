%%%%%% THIS FILE IS A TEMPLATE. Please copy it to the folder config/USER with the name config_USER_param.m
%%%%%% 
%%%%%% Configuration
%%%%%% config_USER_param
%%%%%% 
%%%%%% Define USER parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-26
%%%%%% Keigo Haji
%
%
% Load configurations for parameters defined by the USER 
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

function [robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_USER_param(robot_param, environment_param, gait_planning_param, control_param, ...
    equilibrium_eval_param, ani_settings, save_settings, plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Environment Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Maximum simulation time [s]
environment_param.time_max = 2;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Control Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if contains(robot_param.robot_type, 'HubRobo_v3')
    %%% Controller Proportional (P) gain
    control_param.kp = 10.0;
    %%% Controller Derivative (D) gain
    control_param.kd = 0.3;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Animation Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save_settings.csv_file = 'off';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gripper Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Map Parameters and settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Matching Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing Camera Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end