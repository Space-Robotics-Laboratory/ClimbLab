%%%%%% Configuration
%%%%%% config_equilibrium_param
%%%%%% 
%%%%%% Configure default equilibrium parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-02-22
%%%%%% Kentaro Uno
%
%
% Load default configurations for equilibrium evaluation methods
%
% Function variables:
%
%     OUTPUT
%         equilibrium_eval_param
%     INPUT
%         -

function equilibrium_eval_param = config_equilibrium_param()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Equilibrium evaluation method ('none', 'tsm', 'gia', 'tsm_and_gia')
equilibrium_eval_param.type = 'tsm';
   
end