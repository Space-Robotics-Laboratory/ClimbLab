%%%%%% Update
%%%%%% upd_stop_goal_any
%%%%%% 
%%%%%% Simulation stopper goal
%%%%%% 
%%%%%% Created 2021-04-13
%%%%%% Warley Ribeiro
%%%%%% Last updated 2021-05-04
%%%%%% Keigo Haji

%
%
% Stop simulation if the distance to Goal falls below a certain threshold 
%
% Function variables:
%
%     OUTPUT
%         simu_flag           : Simulation stop if flag is false (logical)
%     INPUT
%         environment_param   :Parameters for environment (class)
%         gait_planning_param : Parameters for gait (class)
%         SV                  : State Variables (SpaceDyn class)

function simu_flag = upd_stop_goal_any(simu_flag, environment_param, gait_planning_param, SV)

if strcmp(environment_param.sim_stop_goal_any,'on')
    % Find the distance to the last goal, when there are multiple goals,
    distance_goal = norm(SV.R0(1:2) - gait_planning_param.goal(1:2,end));
    if distance_goal < environment_param.sim_stop_goal_threshold 
        simu_flag = false;
        disp("STOP: Robot reached goal position")
    end
end