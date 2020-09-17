%%%%%% Save
%%%%%% sav_data_file
%%%%%% 
%%%%%% Save data.csv file
%%%%%% 
%%%%%% Created 2020-05-13
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-05-13
%
%
% Create .csv file to save simulation data 
%
%     Function variables:
%
%     OUTPUT
%         data            : Data saved (table)
%
%     INPUT
%         variables_saved : Collection of all variables to be saved (struct)
%         save_settings   : Data saving settings (struct)
%         run_cod         : Program identification code (string)
%         run_id          : Run identification (string)
%         run_date        : Run identification date (string)
 

function data = sav_data_file(variables_saved, save_settings, run_cod, run_id, run_date)

%%% Definition of files for data file %%%
if strcmp(save_settings.csv_file,'on')
    % Create directory
    dir_name = ['dat/' run_cod '/' run_id];
    mkdir(dir_name);
    % Convert to table and save csv
    data = struct2table(variables_saved);
    writetable(data,[dir_name '/' run_date '_data.csv']);
else
    data = [];
end

end