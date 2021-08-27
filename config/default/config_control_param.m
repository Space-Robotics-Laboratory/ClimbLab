%%%%%% Configuration
%%%%%% config_control_param
%%%%%% 
%%%%%% Configure default control parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
%
%
% Load default configurations for control methods
%
% Function variables:
%
%     OUTPUT
%         control_param
%     INPUT
%         -

function control_param = config_control_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Controller type ('do_nothing', 'torque_PD')
control_param.type = 'torque_PD';


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PD controller settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Controller Proportional (P) gain
control_param.kp = 3.00;
%%% Controller Derivative (D) gain
control_param.kd = 0.02;
   
end