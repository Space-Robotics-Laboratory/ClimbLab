%%%%%% Configuration
%%%%%% config_save_settings
%%%%%% 
%%%%%% Configure default saving settings
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-06
%%%%%% Keigo Haji
%
%
% Load default configurations for saving
%
% Function variables:
%
%     OUTPUT
%         save_settings
%     INPUT
%         -

function save_settings = config_save_settings(environment_param)

%%% Time interval for saving variables (should be larger than time-step)
save_settings.variable_saving_time_interval = environment_param.time_step;

%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';

%%% Save the loaded config file in the dat folder
save_settings.config_file = 'on';

%%% Save TSM variables to csv file on/off
save_settings.tsm = 'off';
%%% Save variables to csv file on/off
save_settings.gia = 'off';
%%% Save manipulability to csv file on/off
save_settings.manipulability = 'off';
    %%% Save mean manipulability of all limbs to csv file on/off
    save_settings.mean_manipulability = 'off';
    %%% Save minimum manipulability of all limbs to csv file on/off
    save_settings.min_manipulability = 'off';
%%% Save Maximum of absolute torque of all joint
save_settings.joint_max_torque = 'off';
%%% Save Root Mean Square (RMS) of torque of all joint (*need Signal Processing Toolbox)
save_settings.joint_rms_torque = 'off';
