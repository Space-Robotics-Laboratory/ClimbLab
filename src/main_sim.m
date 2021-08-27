%%%%%% MAIN
%%%%%% main_sim
%%%%%% 
%%%%%% Main file for climbing/walking simulation
%%%%%% 
%%%%%% Created: 2020-02-20 
%%%%%% by Warley Ribeiro
%%%%%% Last update: 2021-08-23
%%%%%% by Kentaro Uno

clc; clear; close all; 
tic; ini_path(); variables_saved = []; evaluation_param = []; trajectories = [];

%%% Select configuration:
%%% - 'default', 
%%% - 'USER', 
%%% - 'example_demo_1',
%%% - 'example_demo_2',
%%% - 'example_demo_3',
%%% - 'crawl_gait_for_discrete_footholds', 
%%% - 'nonperiodic_gait_for_discrete_footholds', 
%%% - 'gia_static',
%%% - 'iSAIRAS_2020_demo'
%%% - 'clawar_2021_dynamic_climbing_demo'
%%% - 'base_pos_opt_quasistatic' % this is now under development
%%% - 'hubrobo_testfield_exp'
config = 'default';

%%% Load all initial parameters from configuration files stored in config/
[robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_simulation(config);

%%% Define a code for current set of simulations
run_cod = 'test';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize environment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global d_time; global Gravity; global Ez; global contact_f; global x; global y; global z;

ini_environment(environment_param);
surface_param = ini_surface(environment_param);
surface_param = ini_graspable_points(environment_param, surface_param, gripper_param, map_param, matching_settings);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[LP, SV, des_SV, POS_e, ORI_e, cont_POS, ani_settings] = ini_robot(robot_param, gait_planning_param, surface_param, ani_settings);
shape_robot = vis_create_robot_model(ani_settings, LP);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize sensing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
surface_param = ini_sensed_graspable_points(SV, surface_param, sensing_camera_param);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize gait %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[gait_planning_param, motion_planning_param] = ini_gait(gait_planning_param, SV);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[run_id, run_date] = ini_id(robot_param, environment_param, gait_planning_param, control_param);
video   = ini_video_file(ani_settings, run_cod, run_id, run_date);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Simulation loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for time = 0:d_time:environment_param.time_max
	% Display time
	disp(time)
    
%     % @TODO: this will also be implemented as a optional switch and
%     % simulation stopper function
%     if norm(SV.R0(1:2) - gait_planning_param.goal(1:2,gait_planning_param.goal_num)) < 0.05
%         break
%     end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Sensing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    surface_param = upd_sensed_graspable_points(SV, surface_param, sensing_camera_param);
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Planning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	gait_planning_param = upd_gait_planning(gait_planning_param, surface_param, des_SV, SV, LP, POS_e, ...
                                            environment_param, robot_param, sensing_camera_param,time);
   
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Motion Planning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	[motion_planning_param, des_SV] = upd_motion_planning(motion_planning_param, gait_planning_param, ...
				                                          LP, SV, des_SV, cont_POS, POS_e, time);

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Evaluation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	equilibrium_eval_param = upd_equilibrium_eval(equilibrium_eval_param, surface_param, ani_settings, LP, SV, POS_e);


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	SV = upd_control(SV, des_SV, control_param);
	
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Kinematics and dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	SV = upd_grasp_detach(LP, SV, des_SV, environment_param, equilibrium_eval_param, variables_saved);
    cont_POS = upd_collision_endeffector(LP, SV, POS_e, cont_POS);
	SV = upd_ground_reaction_forces_spring_damper(LP, SV, surface_param, POS_e, cont_POS);
	SV = upd_fwd_dynamics(environment_param, LP, SV, des_SV);
	[POS_e, Qe_deg, Q0_deg, SV] = upd_fwd_kinematics(LP, SV);

    evaluation_param = upd_manipulability(LP,SV,evaluation_param);
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save selected variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	variables_saved = upd_variables_saved(variables_saved, save_settings, time, LP, SV, POS_e, environment_param, equilibrium_eval_param, evaluation_param, gait_planning_param);
    
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Climbing animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Visualization
    if rem(time,1/ani_settings.frame_rate) == 0 && strcmp(ani_settings.display,'on')
        inc = environment_param.inc;
    	figure(1); clf; hold on;
        vis_robot(LP, SV, POS_e, shape_robot, inc, ani_settings);
        ani_settings = vis_animation_range(motion_planning_param,surface_param,ani_settings,inc);
        vis_surface(inc, ani_settings, environment_param, surface_param);
        vis_graspable_points(surface_param,inc,ani_settings);
        vis_graspable_points_in_reachable_area(gait_planning_param, inc, ani_settings);
        vis_reachable_area(SV, LP, gait_planning_param, inc, ani_settings, surface_param);
        vis_support_triangle(SV,LP,POS_e,inc,ani_settings);
        gait_planning_param = vis_com_projection(LP, SV, inc, ani_settings, gait_planning_param);
        vis_next_desired_position(gait_planning_param, inc, ani_settings);
        vis_stability_polyhedron(inc, equilibrium_eval_param, ani_settings);
        vis_vectors(inc,ani_settings,equilibrium_eval_param,LP,SV,POS_e,gait_planning_param,sensing_camera_param);
		vis_animation_settings(ani_settings,surface_param,time);
        vis_com_proj_trajectory(gait_planning_param, ani_settings, inc);
        vis_trajectory_4legged(environment_param, motion_planning_param, ani_settings);
        vis_sensing_camera_fov(SV, inc, sensing_camera_param, ani_settings, surface_param);
        vis_sensed_graspable_points(surface_param, inc, ani_settings);
        vis_goal(gait_planning_param, inc, ani_settings);
        writeVideo(video,getframe(1));
%         %%% if you want to save the snapshots at the specific time period, you can uncomment the following code. --> @TODO: this should be selectable. 
%         % @TODO: this will also be implemented as a optional switch 
%         if rem(time, 10) == 0
%             % Create directory
%             if time == 0
%                 dir_name = ['dat/' run_cod '/' run_id];
%                 mkdir(dir_name);
%             end
%             saveas(gcf, [ 'dat/' run_cod '/' run_id '/' num2str(time) 'sec.png'], 'png')
%             saveas(gcf, [ 'dat/' run_cod '/' run_id '/' num2str(time) 'sec.fig'], 'fig')
%         end
    end
    
    %%% Simulation break
    if upd_stop_sim(environment_param, gait_planning_param, SV, des_SV, LP, time) == false
        break
    end
    
    %%% Update Goal Position 
    gait_planning_param = upd_goal_position(gait_planning_param, environment_param, LP, SV);
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Saving variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close(video);
data = sav_data_file(variables_saved, save_settings, run_cod, run_id, run_date, config);
% Plot graphs
vis_plot_graph(data, plot_settings, LP, inc, surface_param, gait_planning_param);

toc
% EOF