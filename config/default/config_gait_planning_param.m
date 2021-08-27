%%%%%% Configuration
%%%%%% config_gait_planning_param
%%%%%% 
%%%%%% Configure default gait parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-07
%%%%%% Keigo Haji
%
%
% Load default configurations for gait methods
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param
%     INPUT
%         -

function gait_planning_param = config_gait_planning_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Gait type ('do_nothing', 'periodic_crawl_gait,' 'crawl_fixed_stride','crawl_gait_for_discrete_footholds','nonperiodic_gait_for_discrete_footholds','GIA_opt_based_pose_planner')
gait_planning_param.type = 'crawl_fixed_stride';


%%% Goal position [m] (3*N matrix)
gait_planning_param.goal(:,1) = [0.4;0;0];

%%% Kinematics feasiblity check switch
gait_planning_param.feasibility_check_excluding_same_graspable_point_switch = 'on';
gait_planning_param.feasibility_check_number_of_legs = 5; % 4 or 5


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Crawl gait settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Period of one cycle [s]
gait_planning_param.T = 4;
%%% Duty cycle
gait_planning_param.beta = 0.75;
%%% Horizontal distance per step (stride) [m]
gait_planning_param.step_length = 0.05;
%%% Step height [m]
gait_planning_param.step_height = 0.03;
%%% Walking sequence: 
% If the gait_type is set as 'crawl' and the walking sequense is not
% defined in your config file such as preset or USER, the walking sequence
% is set [2; 1; 3; 4] as default in ini_gait.m.
gait_planning_param.sequence = []; 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% gait for discrete footholds settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Allowable maximum stride [m] -- If you set this too large, it might 
%%% not able to pass the kinematic feasibility. If you already limit the
%%% allowable stride, it helps with saving the computational time.
gait_planning_param.allowable_max_stride = 0.1; 

   
end