%%%%%% Update
%%%%%% upd_stop_joint_limit
%%%%%% 
%%%%%% Simulation stopper joint limit
%%%%%% 
%%%%%% Created 2021-04-13
%%%%%% Warley Ribeiro

%
%
% Stop simulation if the joint position is outside joint limits
%
% Function variables:
%
%     OUTPUT
%         simu_flag           : Simulation stop if flag is false (logical)
%     INPUT
%         environment_param   : Parameters for environment (class)
%         SV                  : State Variables (SpaceDyn class)

function simu_flag = upd_stop_joint_limit(simu_flag, environment_param, LP, SV)

if strcmp(environment_param.sim_stop_joint_limit,'on')
    for i = 1:LP.num_limb
        joint_limit(3*i-2:3*i-0) = [deg2rad(LP.joint_limit(1,1)) < SV.q(3*i-2) &&  SV.q(3*i-2) < deg2rad(LP.joint_limit(1,2));
                              deg2rad(LP.joint_limit(2,1)) < SV.q(3*i-1) &&  SV.q(3*i-2) < deg2rad(LP.joint_limit(2,2));
                              deg2rad(LP.joint_limit(3,1)) < SV.q(3*i-0) &&  SV.q(3*i-2) < deg2rad(LP.joint_limit(2,2))];
    end
    if sum(joint_limit) < LP.num_q
        simu_flag = false;
        disp("STOP: Configuration is outside joint limit");
    end
        
end