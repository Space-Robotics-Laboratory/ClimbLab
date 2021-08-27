%%%%%% Update
%%%%%% upd_stop_time_max
%%%%%% 
%%%%%% Simulation stopper time
%%%%%% 
%%%%%% Created 2021-04-13
%%%%%% Warley Ribeiro

%
%
% Stop simulation if the simulation time reaches maximum value
%
% Function variables:
%
%     OUTPUT
%         simu_flag           : Simulation stop if flag is false (logical)
%     INPUT
%         environment_param   : Parameters for environment (class)
%         time                : Simulation time [s] (scalar)

function simu_flag = upd_stop_time_max(simu_flag, environment_param, time)

if strcmp(environment_param.sim_stop_time,'on')
    if time >= environment_param.time_max
        simu_flag = false;
        disp("STOP: Maximum time")
    end
end