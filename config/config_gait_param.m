%%%%%% Configuration
%%%%%% config_gait_param
%%%%%% 
%%%%%% Configure default gait parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
%
%
% Load default configurations for gait methods
%
% Function variables:
%
%     OUTPUT
%         gait_param
%     INPUT
%         -

function gait_param = config_gait_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Gait type ('do_nothing', 'crawl_fixed_stride','crawl_uno_ver')
gait_param.type = 'crawl_fixed_stride';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Crawl gait settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Period of one cycle [s]
gait_param.T = 4;
%%% Duty cycle
gait_param.beta = 0.75;
%%% Horizontal distance per step (stride) [m]
gait_param.step_length = 0.05;
%%% Step height [m]
gait_param.step_height = 0.03;
%%% Walking sequence: 1 - 3 - 4 - 2
gait_param.sequence = [1; 3; 4; 2];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Uno gait settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Goal position [m]
gait_param.goal = [0.4;0;-0.08];
   
end