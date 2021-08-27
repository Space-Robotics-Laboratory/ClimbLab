%%%%%% Update
%%%%%% upd_stop_goal_grasping
%%%%%% 
%%%%%% Simulation stopper goal
%%%%%% 
%%%%%% Created 2021-04-13
%%%%%% Warley Ribeiro

%
%
% Stop simulation if the distance to Goal falls below a certain threshold and all legs are grasping
%
% Function variables:
%
%     OUTPUT
%         simu_flag           : Simulation stop if flag is false (logical)
%     INPUT
%         environment_param   :Parameters for environment (class)
%         gait_planning_param : Parameters for gait (class)
%         SV                  : State Variables (SpaceDyn class)
%         LP                  : Link parameters (SpaceDyn class)

function simu_flag = upd_stop_goal_grasping(simu_flag, environment_param, gait_planning_param, LP, SV)

if strcmp(environment_param.sim_stop_goal_grasping,'on')
    distance_goal = norm(SV.R0(1:2) - gait_planning_param.goal(1:2,end));
    if distance_goal < environment_param.sim_stop_goal_threshold && sum(SV.sup) == LP.num_limb
        simu_flag = false;
        disp("STOP: Robot reached goal position")
    end
end