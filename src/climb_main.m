%%%%%% MAIN
%%%%%% climb_main
%%%%%% 
%%%%%% Main file for climbing/walking simulation
%%%%%% 
%%%%%% Created 2020-02-20 by Warley Ribeiro
%%%%%% Last update: 2020-10-16 by Kentaro Uno

clc; clear; close all; 
tic; ini_path(); variables_saved = [];

%%% Select configuration 'default', 'USER', 'uno_crawl_param', 'nonperiodic_demo_param', 'gia_static', 'iSAIRAS_2020_demo_param'
config = 'iSAIRAS_2020_demo_param';
%%% Load all initial parameters from configuration files stored in config/
[robot_param, environment_param, gait_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings] = config_simulation(config);

%%% Define a code for current set of simulations
run_cod = 'test';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize environment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global d_time; global Gravity; global Ez; global contact_f; global x; global y; global z;

ini_environment(environment_param);
surface_param = ini_surface(environment_param);
surface_param = ini_graspable_points(environment_param, surface_param, gripper_param, map_param, matching_settings);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[LP, SV, des_SV, F_grip, POS_e, ORI_e, cont_POS] = ini_robot(robot_param, gait_param, surface_param);
shape_robot = vis_create_robot_model(ani_settings, LP, F_grip);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize gait %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[gait_param, path_planning_param, motion_planning_param] = ini_gait(gait_param);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[run_id, run_date] = ini_id(robot_param, environment_param, gait_param, control_param);
video   = ini_video_file(ani_settings, run_cod, run_id, run_date);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Simulation loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for time = 0:d_time:environment_param.time_max
	% Display time
	disp(time)
    
%     if norm(SV.R0(1:2) - gait_param.goal(1:2)) < 0.02
%         break
%     end

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Path Planning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	path_planning_param = upd_path_planning(path_planning_param, gait_param, surface_param, des_SV, SV, LP, POS_e, ...
                                            environment_param, robot_param, time);
   
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Motion Planning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	[motion_planning_param, des_SV] = upd_motion_planning(motion_planning_param, gait_param, ...
				                                            path_planning_param, LP, SV, des_SV, cont_POS, time);

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Evaluation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	equilibrium_eval_param = upd_equilibrium_eval(equilibrium_eval_param, surface_param, LP, SV, POS_e, F_grip);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	SV = upd_control(SV, des_SV, control_param);
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Kinematics and dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	cont_POS = upd_collision_endeffector(LP, SV, POS_e, cont_POS);
	SV = upd_ground_reaction_forces_spring_damper(LP, SV, surface_param, POS_e, cont_POS);
	SV = upd_fwd_dynamics(environment_param, LP, SV, des_SV);
	[POS_e, Qe_deg, Q0_deg, SV] = upd_fwd_kinematics(LP, SV);

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save selected variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	variables_saved = upd_variables_saved(variables_saved, save_settings, time, LP, SV, POS_e, equilibrium_eval_param);
    
    SV = upd_grasp_detach(LP, SV, des_SV, F_grip, environment_param, equilibrium_eval_param, variables_saved);
% 	SV.sup = des_SV.sup;
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Climbing animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Visualization
    if rem(time,1/ani_settings.frame_rate) == 0 && strcmp(ani_settings.display,'on')
        inc = environment_param.inc;
    	figure(1); clf; hold on;
        vis_robot(LP, SV, F_grip, POS_e, shape_robot, inc, ani_settings);
        ani_settings = vis_animation_range(motion_planning_param,surface_param,ani_settings,inc);
        vis_surface(inc, ani_settings);
        vis_graspable_points(surface_param,inc,ani_settings);
        vis_graspable_points_in_reachable_area(path_planning_param, inc, ani_settings);
        vis_reachable_area(SV, LP, path_planning_param, inc, ani_settings, surface_param);
        vis_goal(gait_param, inc, ani_settings);
        vis_support_triangle(SV,POS_e,inc,ani_settings);
        vis_com_projection(LP, SV, inc, ani_settings);
        vis_next_desired_position(path_planning_param, inc, ani_settings);
        vis_stability_polyhedron(inc, equilibrium_eval_param, ani_settings);
        vis_vectors(inc,ani_settings,equilibrium_eval_param,LP,SV);
		vis_animation_settings(ani_settings,surface_param,time);
        vis_trajectory_4legged(environment_param,ani_settings,variables_saved);
        writeVideo(video,getframe(1));
    end

end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Saving variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close(video);
data = sav_data_file(variables_saved, save_settings, run_cod, run_id, run_date);
% Plot graphs
vis_plot_graph(data, plot_settings, LP, inc, surface_param, path_planning_param);

toc
% EOF