%%%%%% Update
%%%%%% upd_variables_saved
%%%%%% 
%%%%%% Create variables to save
%%%%%% 
%%%%%% Created 2020-05-12
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-05-12
%
%
% Create identification for file naming
%
%     Function variables:
%
%     OUTPUT
%         variables_saved        : Variables to be saved (struct)
%     INPUT
%         time				     : Simulation time [s] (scalar) 
%         equilibrium_eval_param : Parameters for equilibrium evaluation (struct)

function variables_saved = upd_variables_saved(variables_saved, save_settings, time, SV, equilibrium_eval_param)

if strcmp(save_settings.csv_file,'on')
    if time == 0
        cnt = 1;
    else
        cnt = size(variables_saved.time,1) + 1;
    end
    % Save time
    variables_saved.time(cnt,1) = time;
    % Save Base position
    variables_saved.R0(cnt,:) = SV.R0';
    % Save Base attitude
    variables_saved.Q0(cnt,:) = SV.Q0';
    % Save Joints angular position
    variables_saved.q(cnt,:) = SV.q';
    
    % Save TSM
    if strcmp(save_settings.tsm,'on')
        variables_saved.tsm(cnt,1) = equilibrium_eval_param.tsm;
    end
    
    % Save GIA
    if strcmp(save_settings.gia,'on')
        variables_saved.gia(cnt,:) = equilibrium_eval_param.gia';
        variables_saved.inclination_margin(cnt,:) = equilibrium_eval_param.inclination_margin;
        variables_saved.acc_margin(cnt,:) = equilibrium_eval_param.acc_margin';
    end

end