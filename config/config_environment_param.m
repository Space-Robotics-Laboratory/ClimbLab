%%%%%% Configuration
%%%%%% config_environment_param
%%%%%% 
%%%%%% Configure default environment parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
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

%%% Gripper detachment method ('none', 'max_holding_force')
environment_param.detachment_detection_method = 'none';

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