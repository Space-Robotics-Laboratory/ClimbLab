%%%%%% Update
%%%%%% upd_variables_saved
%%%%%% 
%%%%%% Create variables to save
%%%%%% 
%%%%%% Created 2020-05-12
%%%%%% Warley Ribeiro
%%%%%% Last update: 2021-07-09
%%%%%% Kentaro Uno
%
%
% Save history of selected variables
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
%         environment_param.time_step : time step of the simulation
%         equilibrium_eval_param : Parameters for equilibrium evaluation (struct)
%         evaluation_param       : Parameters for evaluation (struct)

function variables_saved = upd_variables_saved(variables_saved, save_settings, time, LP, SV, POS_e, environment_param, equilibrium_eval_param, evaluation_param, gait_planning_param)

if strcmp(save_settings.csv_file,'on')
    if time == 0
        cnt = 1;
        variables_saved = save_variables(cnt, variables_saved, save_settings, time, LP, SV, POS_e, equilibrium_eval_param, evaluation_param, gait_planning_param);
    
    % With default settings, all variables are saved at each calc cycle
    elseif save_settings.variable_saving_time_interval == environment_param.time_step
        cnt = size(variables_saved.time,1) + 1;
        variables_saved = save_variables(cnt, variables_saved, save_settings, time, LP, SV, POS_e, equilibrium_eval_param, evaluation_param, gait_planning_param);
        
    % In case save_settings.variable_saving_time_interval > time_step, all
    % variables are only logged every when the saving time interval is passed.
    elseif save_settings.variable_saving_time_interval > environment_param.time_step
        if round(rem(time, save_settings.variable_saving_time_interval),6) == environment_param.time_step % just rem is not enough to adjust the accuracy of the output
            cnt = size(variables_saved.time,1) + 1;
            variables_saved = save_variables(cnt, variables_saved, save_settings, time, LP, SV, POS_e, equilibrium_eval_param, evaluation_param, gait_planning_param);
        end 
    else
        error("In upd_variables_saved(): save_settings.variable_saving_time_interval should be larger than nvironment_param.time_step !!!");
    end
end

end

function variables_saved = save_variables(cnt, variables_saved, save_settings, time, LP, SV, POS_e, equilibrium_eval_param, evaluation_param, gait_planning_param)
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

    % Save Maximum of absolute torque of all joint
    if strcmp(save_settings.joint_max_torque,'on')
        variables_saved.tau_max(cnt,1) = max( abs(SV.tau') ); 
    end

    % Save Root Mean Square (RMS) of torque of all joint (*need Signal Processing Toolbox)
    if strcmp(save_settings.joint_rms_torque,'on')
        variables_saved.tau_rms(cnt,1) = rms(SV.tau'); 
    end

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
        variables_saved.gia_inclination_margin(cnt,:) = equilibrium_eval_param.gia_inclination_margin;
        variables_saved.gia_margin(cnt,:) = equilibrium_eval_param.gia_margin';
    end

    % Save Manipulability Measure 
    if strcmp(save_settings.manipulability,'on')
        for i = 1:size(evaluation_param.manipulability,2)
            variables_saved.(genvarname(['manipulability' num2str(i)]))(cnt,1) = evaluation_param.manipulability(i);
        end
    end
    % Save the mean value of the Manipulability Measure
    if strcmp(save_settings.mean_manipulability,'on')
        variables_saved.mean_manipulability(cnt,1) = mean( evaluation_param.manipulability ); 
    end
    
    % Save the minimum value of the Manipulability Measure
    if strcmp(save_settings.min_manipulability,'on')
        variables_saved.min_manipulability(cnt,1) = min( evaluation_param.manipulability ); 
    end

end