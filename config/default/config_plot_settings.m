%%%%%% Configuration
%%%%%% config_plot_settings
%%%%%% 
%%%%%% Configure default plotting settings
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-06
%%%%%% Keigo Haji
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
%%% Plot Base orientation
plot_settings.base_ori = 'off';
plot_settings.base_ori_fig_number = 3;
%%% Plot Base velocity
plot_settings.base_vel = 'off';
plot_settings.base_vel_fig_number = 4;
%%% Plot Base angular velocity
plot_settings.base_angvel = 'off';
plot_settings.base_angvel_fig_number = 5;
%%% Plot Base velocity
plot_settings.base_acc = 'off';
plot_settings.base_acc_fig_number = 6;
%%% Plot Base angular velocity
plot_settings.base_angacc = 'off';
plot_settings.base_angacc_fig_number = 7;

%%% Plot Joint angular position
plot_settings.joint_pos = 'off';
plot_settings.joint_pos_fig_number = 8;
%%% Plot Joint angular velocity
plot_settings.joint_vel = 'off';
plot_settings.joint_vel_fig_number = 9;
%%% Plot Joint angular acceleration
plot_settings.joint_acc = 'off';
plot_settings.joint_acc_fig_number = 10;
%%% Plot Joint torque
plot_settings.joint_torque = 'off';
plot_settings.joint_torque_fig_number = 11;

%%% Plot Leg Position
plot_settings.leg_pos = 'off';
plot_settings.leg_pos_fig_number = 12; % and 13, 14, 15 are also used 

%%% Plot Reaction Force
plot_settings.reaction_force = 'off';
plot_settings.reaction_force_fig_number = 16; % and 17~23  are also used 


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 101 ~ 200
%%% Plot footholds history on/off
plot_settings.footholds = 'off';
plot_settings.footholds_fig_number = 101;


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 201 ~ 300
%%% Plot TSM (To turn ON, requires equilibrium evaluation method as 'tsm')
plot_settings.tsm = 'off';
plot_settings.tsm_fig_number = 201;

%%% Plot GIA (Acceleration) margin (To turn ON, requires equilibrium evaluation method as 'gia')
plot_settings.gia_margin = 'off';
plot_settings.gia_margin_fig_number = 202;

%%% Plot GIA Inclination margin (To turn ON, requires equilibrium evaluation method as 'gia')
plot_settings.gia_inclination_margin = 'off';
plot_settings.gia_inclination_margin_fig_number = 203;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Evaluation related %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 301 ~ 400
%%% Plot manipulability
plot_settings.manipulability = 'off';
plot_settings.manipulability_fig_number = 301;
plot_settings.mean_manipulability = 'off';
plot_settings.mean_manipulability_fig_number = 302;
plot_settings.min_manipulability = 'off';
plot_settings.min_manipulability_fig_number = 303;

%%% Plot Max absolute torque of all joint
plot_settings.joint_max_torque = 'off';
plot_settings.joint_max_torque_fig_number = 310;

%%% Plot RMS torque of all joint
plot_settings.joint_rms_torque = 'off';
plot_settings.joint_rms_torque_fig_number = 311;

end