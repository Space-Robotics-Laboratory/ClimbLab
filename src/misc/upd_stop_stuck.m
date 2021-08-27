%%%%%% Update
%%%%%% upd_stop_stuck
%%%%%% 
%%%%%% Simulation stopper stuck
%%%%%% 
%%%%%% Created 2021-04-13
%%%%%% Warley Ribeiro
%%%%%% Lase updated: 2021-04-20
%%%%%% Keigo Haji
%
%
% Stop simulation if the robot selects the same position for grasping point 
%
% Function variables:
%
%     OUTPUT
%         simu_flag           : Simulation stop if flag is false (logical)
%     INPUT
%         environment_param   :Parameters for environment (class)
%         gait_planning_param : Parameters for gait (class)
%         LP                    : Link parameters (SpaceDyn class)
%         des_SV                : Desired state values (SpaceDyn class)

function simu_flag = upd_stop_stuck(simu_flag, environment_param, gait_planning_param, LP,des_SV)
    
if strcmp(environment_param.sim_stop_stuck,'on')
    
    % If all legs are in support phase
    if sum(des_SV.sup) == LP.num_limb
    
        % Initialize next positions of limbs
        des_pos_of_limbs = zeros(3,4);
        
        % Insert current counts of footholds
        count = gait_planning_param.footholds_count_limb;
        
        % Get the number of how many steps robot walked in total
        m = length(gait_planning_param.swing_number_history);
        
        % Get the desired positions of limbs from footholds_history_limb
        for i = 1:LP.num_limb
            des_pos_of_limbs(:,i) = gait_planning_param.footholds_history_limb(:,gait_planning_param.footholds_count_limb(i), i);
        end
        
        % Prepare the matrix representing previous limbs' positions
        previous_pos_of_limbs = des_pos_of_limbs;
        
        % Check previous phases 
        for j = 1 : environment_param.sim_stop_stuck_serach_range_threshold
            % Check if the phase can be traced back     
            if m - j >= 0
                % Get the previous swing limb number
                swing_limb_num = gait_planning_param.swing_number_history(m-j+1);
                % Update the previous positon of swing limb
                previous_pos_of_limbs(:,swing_limb_num) = gait_planning_param.footholds_history_limb(:,count(swing_limb_num)-1,swing_limb_num);
                % If the positions of the limbs are too close, we jduged
                % the robot gets stuck.
                % [Note] norm(X,1) means max(sum(abs(X)))
                if norm(des_pos_of_limbs - previous_pos_of_limbs, 1) < environment_param.sim_stop_stuck_threshold
                    simu_flag = false;
                    disp("STOP: Robot is stuck")
                end
                % Update count to trace back     
                count(swing_limb_num) = count(swing_limb_num) - 1;
            end
        end
    end
    
end



