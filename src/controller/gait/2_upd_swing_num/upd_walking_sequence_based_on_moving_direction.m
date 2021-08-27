%%%%%% Update
%%%%%% upd_walking_sequence_based_on_moving_direction
%%%%%%
%%%%%% Update walking sequence based on moving direction
%%%%%%
%%%%%% Created: 2021-05-04
%%%%%% Keigo Haji
%%%%%% Last updated: 
%%%%%% -
%
%
% Update walking sequence for crawl gait based on moving direction
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for path planning (class)
%
%         gait_planning_param.sequence      
%
%     INPUT
%         gait_planning_param   : Parameters for gait planning (class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_walking_sequence_based_on_moving_direction(gait_planning_param, des_SV, SV, LP, time)

% If all legs are in support phase
if sum(des_SV.sup) == LP.num_limb
    % Plan before starting the hind leg swing period
    if strcmp(gait_planning_param.crawl_gait_direction_change, 'on') && gait_planning_param.swing_number == gait_planning_param.sequence(1)
       
        % Set the Vector_Base_to_Goal
        vec_bg = gait_planning_param.goal(:,gait_planning_param.goal_num) - SV.R0;
                
        %%% When the new sequence starts, Re-set the Walking sequence for
        %%% crawl gait based on the current goal direction
        cross_vec_tmp1 = cross([1,1,0],[vec_bg(1),vec_bg(2),0]);
        if cross_vec_tmp1(3) < 0
            cross_vec_tmp2 = cross([1,-1,0],[vec_bg(1),vec_bg(2),0]);
            if cross_vec_tmp2(3) < 0
                % Limb 1&2 are hind legs
                gait_planning_param.sequence = [1; 4; 2; 3];
            elseif cross_vec_tmp2(3) >= 0
                % Limb 2&3 are hind legs
                gait_planning_param.sequence = [2; 1; 3; 4];
            end
        elseif cross_vec_tmp1(3) >= 0
            cross_vec_tmp2 = cross([1,-1,0],[vec_bg(1),vec_bg(2),0]);
            if cross_vec_tmp2(3) < 0
                % Limb 1&4 are hind legs
                gait_planning_param.sequence = [4; 3; 1; 2];
            elseif cross_vec_tmp2(3) >= 0
                % Limb 3&4 are hind legs
                gait_planning_param.sequence = [3; 2; 4; 1];
            end
        end
        % If the new sequence is diffent from previous sequence, we
        % need to stop swinging limb once to adjsut base position.
        if gait_planning_param.swing_number ~= gait_planning_param.sequence(1) && contains(gait_planning_param.type, 'crawl') 
            gait_planning_param.crawl_gait_sequence_change_flag = true;
            % The swing limb number is set as the last number of the new
            % sequence. This is because the first swing leg of the sequence
            % is selected in the next period, since the swing leg is the
            % next index in the previous sequence.
            gait_planning_param.swing_number = gait_planning_param.sequence(end);
        else
            gait_planning_param.crawl_gait_sequence_change_flag = false;
            gait_planning_param.swing_number = gait_planning_param.sequence(1);
        end
    end
end
end

