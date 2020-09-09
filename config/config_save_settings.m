%%%%%% Configuration
%%%%%% config_save_settings
%%%%%% 
%%%%%% Configure default saving settings
%%%%%% 
%%%%%% Created 2020-07-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-08
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

function save_settings = config_save_settings()

%%% Save basic variables to csv file on/off
save_settings.csv_file = 'on';

%%% Save TSM variables to csv file on/off
save_settings.tsm = 'off';
%%% Save variables to csv file on/off
save_settings.gia = 'off';
