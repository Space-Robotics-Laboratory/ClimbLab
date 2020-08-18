%%%%%% Configuration
%%%%%% config_plot_settings
%%%%%% 
%%%%%% Configure default plotting settings
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
%
%
% Load default configurations for plots
%
% Function variables:
%
%     OUTPUT
%         plot_settings
%     INPUT
%         -

function plot_settings = config_plot_settings()
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% General settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% Font name
plot_settings.font_name = 'Times New Roman';
%%% Font size
plot_settings.font_size = 25;
%%% Line width
plot_settings.width = 3;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Basic graphs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 2 ~ 100
%%% Plot Base position
plot_settings.base_pos = 'on';
plot_settings.base_pos_fig_number = 2;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 101 ~ 200
%%% Plot footholds history on/off
plot_settings.footholds = 'off';
plot_settings.footholds_fig_number = 101;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 201 ~ 300
%%% Plot TSM
plot_settings.tsm = 'off';
plot_settings.tsm_fig_number = 201;

%%% Plot Acceleration margin
plot_settings.acc_margin = 'off';
plot_settings.acc_margin_fig_number = 202;

%%% Plot Inclination margin
plot_settings.inclination_margin = 'off';
plot_settings.inclination_margin_fig_number = 203;
   
end