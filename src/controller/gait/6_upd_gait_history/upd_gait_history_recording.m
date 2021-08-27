%%%%%% Update
%%%%%% upd_gait_history_recording
%%%%%% 
%%%%%% Update gait planning variables history
%%%%%% 
%%%%%% Created: 2021-04-20
%%%%%% Keigo Haji
%%%%%% Last updated: 2021-04-20
%%%%%% Keigo Haji
%
%
% Update the gait planning variables history and record them
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for gait planning (class)
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         des_SV                : Desired state values (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)
%
%
function gait_planning_param = upd_gait_history_recording(gait_planning_param, LP, POS_e, des_SV, time)
% If all legs are in support phase
if sum(des_SV.sup) == LP.num_limb
    % Record the initial footholds of leg i in time == 0
    i = gait_planning_param.swing_number;
    if time == 0
        for j=1:LP.num_limb
            gait_planning_param.footholds_history_limb(:,gait_planning_param.footholds_count_limb(j),j) = POS_e(:,j);
        end
    end
    
    % Update and added the footholds of leg i and count
    gait_planning_param.footholds_count_limb(i) = gait_planning_param.footholds_count_limb(i) + 1;
    gait_planning_param.footholds_history_limb(:,gait_planning_param.footholds_count_limb(i),i) = gait_planning_param.POS_next(:,i);
    gait_planning_param.swing_number_history = horzcat(gait_planning_param.swing_number_history, gait_planning_param.swing_number);

end
end

