%%%%%% Update
%%%%%% upd_stop_singularity
%%%%%% 
%%%%%% Simulation stopper singularity
%%%%%% 
%%%%%% Created 2021-04-13
%%%%%% Warley Ribeiro

%
%
% Stop simulation if the robot is in a singular configuration
%
% Function variables:
%
%     OUTPUT
%         simu_flag           : Simulation stop if flag is false (logical)
%     INPUT
%         environment_param   : Parameters for environment (class)
%         SV                  : State Variables (SpaceDyn class)

function simu_flag = upd_stop_singularity(simu_flag, environment_param, SV)

if strcmp(environment_param.sim_stop_singular,'on')
    if isreal(SV.q) == false
        simu_flag = false;
        disp("STOP: Singularity");
    end
end