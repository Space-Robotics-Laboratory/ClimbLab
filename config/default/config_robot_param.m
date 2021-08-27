%%%%%% Configuration
%%%%%% config_robot_param
%%%%%% 
%%%%%% Configure default robot parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-06-02
%%%%%% Keigo Haji
%
%
% Load default configurations for robot
%
% Function variables:
%
%     OUTPUT
%         robot_param
%     INPUT
%         -

function robot_param = config_robot_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the robot to simulate ('HubRobo_grip_to_spine_old',
%%% 'HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_spine',
%%% 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm',
%%% 'HubRobo_v3_2_grip_to_palm', 'ANYmal_B', 'Climbing_ANYmal','ALPHRED'
robot_param.robot_type = 'HubRobo_v3_2_grip_to_palm';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Position settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% x and y position of legs relative to base center [m]
robot_param.x_foot_dist  = 0.14;
robot_param.y_foot_dist  = 0.14;
%%% Initial x and y "offset" position of the base from the neutral position of the base center [m]
robot_param.x_base_pos_offset_from_the_neutral_pos = 0.0;
robot_param.y_base_pos_offset_from_the_neutral_pos = 0.0;
%%% Height of base relative to map [m]
robot_param.base_height = 0.11;
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0;0];
%%% Base yaw orientation  [deg]
robot_param.initial_yaw = 0;
end