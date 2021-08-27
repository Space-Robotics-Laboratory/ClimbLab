%%%%%% Update
%%%%%% upd_goal_position
%%%%%% 
%%%%%% Update goal position
%%%%%% 
%%%%%% Created 2021-05-03
%%%%%% Keigo Haji
%%%%%% Updated 2021-07-09
%%%%%% Keigo Haji
%
% Update goal position to pass some positions on the map
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param : Parameters for gait (class)
%     INPUT
%         environment_param   :Parameters for environment (class)
%         gait_planning_param : Parameters for gait (class)
%         SV                  : State Variables (SpaceDyn class)
%         LP                  : Link parameters (SpaceDyn class)

function gait_planning_param = upd_goal_position(gait_planning_param, environment_param, LP, SV)

% Execute when multiple goals are set
if size(gait_planning_param.goal, 2) > 1
    i = gait_planning_param.goal_num;
    % Calculate the distance between the robot base position and the
    % current goal position
    distance_goal = norm(SV.R0(1:2) - gait_planning_param.goal(1:2,i));
    
    % If the robot is closer to the goal than the threshold, it is judged
    % to have reached it. 
%     if distance_goal < environment_param.sim_stop_goal_threshold && sum(SV.sup) == LP.num_limb
%     if distance_goal < environment_param.sim_stop_goal_threshold

    % Update the sub-goal position when the robot passes the sub-goal. 
    % This is because the moving field of local path planning doesn't work
    % well if the goal position is updated before the robot passes the
    % goal.
    if gait_planning_param.goal_num < size(gait_planning_param.goal,2)
        % Set a vector from the robot base to the current sub-gaol
        vec_current_goal_to_base = SV.R0 - gait_planning_param.goal(:,gait_planning_param.goal_num);
        unit_vec_current_goal_to_base = vec_current_goal_to_base/norm(vec_current_goal_to_base);
        % Set a vector from the current sub-goal to the next sub-goal
        vec_current_subgoal_to_next_subgoal = gait_planning_param.goal(:,gait_planning_param.goal_num + 1) - gait_planning_param.goal(:,gait_planning_param.goal_num);
        unit_vec_current_subgoal_to_next_subgoal = vec_current_subgoal_to_next_subgoal/norm(vec_current_subgoal_to_next_subgoal);
        % Calculate theta between two vectors
        dot_tmp = dot(unit_vec_current_subgoal_to_next_subgoal(1:2),unit_vec_current_goal_to_base(1:2));
        theta_tmp = acos(dot_tmp);   % 0 to pi
        %     if distance_goal < environment_param.sim_stop_goal_threshold && theta_tmp <= pi/2
        if theta_tmp < pi/2
            % Only in the case of crawl gait, Update the goal judgment for each sequence
            if strfind(gait_planning_param.type, 'crawl') > 0
                if  gait_planning_param.swing_number == gait_planning_param.sequence(end)
                    gait_planning_param.goal_num = i + 1;
                end
            else
                % Set the goal as next one
                gait_planning_param.goal_num = i +1;
            end
        end
    end
end

