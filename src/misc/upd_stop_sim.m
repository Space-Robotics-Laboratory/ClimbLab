%%%%%% Update
%%%%%% upd_stop_sim
%%%%%% 
%%%%%% Simulation stopper 
%%%%%% 
%%%%%% Created 2020-07-23
%%%%%% Koki Kurihara
%%%%%% Last update: 2020-12-18
%%%%%% Warley Ribeiro

%
%
% Stop simulation based on selected configuration parameters
%
% Function variables:
%
%     OUTPUT
%         simu_flag           : Simulation stop if flag is false (logical)
%     INPUT
%         environment_param   :Parameters for environment (class)
%         gait_planning_param : Parameters for gait (class)
%         SV                  : State Variables (SpaceDyn class)
%         des_SV              : State Variables (SpaceDyn class)
%         LP                  : Link parameters (SpaceDyn class)
%         time                : Simulation time [s] (scalar)

function simu_flag = upd_stop_sim(environment_param, gait_planning_param, SV, des_SV, LP, time)

% initialize flag
simu_flag = true;

    % Stop simulation when robot reaches the goal
    simu_flag = upd_stop_goal_grasping(simu_flag, environment_param, gait_planning_param, LP, SV);
    simu_flag = upd_stop_goal_any(simu_flag, environment_param, gait_planning_param, SV);
    % Stop simulation when robot is selecting the same grasping point
	simu_flag = upd_stop_stuck(simu_flag, environment_param, gait_planning_param, LP,des_SV);
    % Stop simulation for singular configuration
    simu_flag = upd_stop_singularity(simu_flag, environment_param, SV);
    % Stop simulation for joint limit
    simu_flag = upd_stop_joint_limit(simu_flag, environment_param, LP, SV);
    % Stop simulation for time limit
    simu_flag = upd_stop_time_max(simu_flag, environment_param, time);
    
end