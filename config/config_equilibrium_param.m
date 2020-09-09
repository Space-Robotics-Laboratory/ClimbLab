%%%%%% Configuration
%%%%%% config_equilibrium_param
%%%%%% 
%%%%%% Configure default equilibrium parameters
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
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

%%% Equilibrium evaluation method ('none', 'tsm', 'gia')
equilibrium_eval_param.type = 'tsm';

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GIA polyhedron settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Plot polyhderon on animation on/off (requires equilibrium evaluation method as 'gia')
equilibrium_eval_param.plot_polyhedron = 'off';
% Transformation from acceleration to plot in the position coordinate
equilibrium_eval_param.expansion_factor = 0.02;
   
end