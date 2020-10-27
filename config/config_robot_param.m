%%%%%% Configuration
%%%%%% config_robot_param
%%%%%% 
%%%%%% Configure default robot parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
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

%%% Type of the robot to simulate ('HubRobo_v2_2_no_grip', 'HubRobo_v2_2_grip_to_palm', 'HubRobo_v3_1_grip_to_palm', 'HubRobo_v2_2_grip_to_spine')
robot_param.robot_type = 'HubRobo_v2_2_grip_to_palm';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Position settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% x and y position of legs relative to base center [m]
robot_param.foot_dist  = 0.12;
%%% Height of base relative to map [m]
robot_param.base_height = 0.12;
%%% Base position [m] 2x1 vector. or 'default' for default setting
robot_param.base_pos_xy = [0;0];
   
end