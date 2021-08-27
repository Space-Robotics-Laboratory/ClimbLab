%%%%%% Configuration
%%%%%% config_environment_param
%%%%%% 
%%%%%% Configure default environment parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-05-04
%%%%%% Keigo Haji
%
%
% Load default configurations for environments
%
% Function variables:
%
%     OUTPUT
%         environment_param
%     INPUT
%         -

function environment_param = config_environment_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Type of the surface ('flat_HR', 'rough', 'flat_003', 'flat_006', 'flat_008', 'flat_009', 'flat_012') 
environment_param.surface_type = 'rough';
%%% Gravity [G]
environment_param.grav = 1/6;

%%% Dynamics on/off
environment_param.dynamics_flag = 'on';

%%% Gripper detachment method ('none', 'tsm', 'max_holding_force')
environment_param.detachment_detection_method = 'none';

%%% Simulation stop
%%% Simulation stop reaching goal when all limbs are grasping
environment_param.sim_stop_goal_grasping = 'on';
%%% Simulation stop reaching goal under any condition
environment_param.sim_stop_goal_any = 'off';
%%% Simulation stop reaching goal threshold
environment_param.sim_stop_goal_threshold = 0.03;
%%% Simulation stop if robot is stuck (selecting same grasping point)
environment_param.sim_stop_stuck = 'off';
%%% Simulation stop stuck threshold
environment_param.sim_stop_stuck_threshold = 0.005;
%%% Simulation stop stuck search range threshold
environment_param.sim_stop_stuck_serach_range_threshold = 4;
%%% Simulation stop for singular configuration
environment_param.sim_stop_singular = 'on';
%%% Simulation stop for joint outside limit
environment_param.sim_stop_joint_limit = 'on';
%%% Simulation stop for maximum time
environment_param.sim_stop_time = 'on';
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Time settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Time-step [s]
environment_param.time_step = 0.001;
%%% Maximum simulation time [s]
environment_param.time_max = 8;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Surface settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Surface Inclination [deg]
environment_param.inc = 0;
%%% Ground reaction force stiffness coefficient
environment_param.surface_K = 1000;
%%% Ground reaction force damping coefficient (scalar)
environment_param.surface_D = 1;
%%% Graspable points detection type ('all', 'gripper', 'peaks')
environment_param.graspable_points_detection_type = 'all';
   
end