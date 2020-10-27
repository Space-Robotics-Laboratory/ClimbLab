%%%%%% Update
%%%%%% upd_variables_saved
%%%%%% 
%%%%%% Create variables to save
%%%%%% 
%%%%%% Created 2020-05-12
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-09-08
%
%
% Create identification for file naming
%
%     Function variables:
%
%     OUTPUT
%         variables_saved        : Variables to be saved (struct)
%     INPUT
%         variables_saved        : Variables to be saved (struct)
%         save_settings          : Data saving settings (struct)
%         time				     : Simulation time [s] (scalar) 
%         LP                     : Link Parameters (SpaceDyn class)
%         SV                     : State Variables (SpaceDyn class)
%         POS_e                  : Position of the end-effector [m] (3xnum_limb matrix)
%         equilibrium_eval_param : Parameters for equilibrium evaluation (struct)

function variables_saved = upd_variables_saved(variables_saved, save_settings, time, LP, SV, POS_e, equilibrium_eval_param)

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
    % Save Base velocity
    variables_saved.v0(cnt,:) = SV.v0';
    % Save Base angular velocity
    variables_saved.w0(cnt,:) = SV.w0';
    % Save Base acceleration
    variables_saved.vd0(cnt,:) = SV.vd0';
    % Save Base angular acceleration
    variables_saved.wd0(cnt,:) = SV.wd0';
    
    % Save Joints angular position
    variables_saved.q(cnt,:) = SV.q';
    % Save Joints angular velocity
    variables_saved.qd(cnt,:) = SV.qd';
    % Save Joints angular acceleration
    variables_saved.qdd(cnt,:) = SV.qdd';
    % Save Joints torque
    variables_saved.tau(cnt,:) = SV.tau';
    
    % Save end-effector position
    for i = 1:size(POS_e,2)
        % Generate name for variables pos_e1, pos_e2, ...
        variables_saved.(genvarname(['pos_e' num2str(i)]))(cnt,:) = POS_e(:,i)';
    end 
    
    % Save end-effector reaction force
    Force = SV.Fe(:,LP.SE == 1);
    for i = 1:size(POS_e,2)
        % Generate name for variables Fe_e1, Fe_e2, ...
        variables_saved.(genvarname(['Fe' num2str(i)]))(cnt,:) = Force(:,i)';
        variables_saved.(genvarname(['Fe_res' num2str(i)]))(cnt,:) = norm(Force(:,i)');
    end
    
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