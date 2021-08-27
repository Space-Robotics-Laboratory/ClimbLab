%%%%%% MAIN
%%%%%% main_itterative_sim
%%%%%% 
%%%%%% Main file for the itterative static analysis for the CLAWAR2021
%%%%%% paper. Maintained by Kentaro Uno (unoken@dc.tohoku.ac.jp).
%%%%%% 
%%%%%% --------------------------------------------------------------------
%%%%%% This configration file can reproduce the simulation result used in
%%%%%% CLAWAR 2021 proceedings paper by K. Uno and G. Valsecchi et al.
%%%%%% Proceedings Paper URL: 
%%%%%% https://******** (to be added)
%%%%%% --------------------------------------------------------------------
%%%%%% 
%%%%%% Created: 2021-02-17 
%%%%%% by Kentaro Uno
%%%%%% Last update: 2021-07-28
%%%%%% by Kentaro Uno

clc; clear; close all; 
tic; ini_path(); variables_saved = []; evaluation_param = [];

%%% Select configuration:
%%% - 'clawar_2021_statistical_analysis'
config = 'clawar_2021_statistical_analysis';

%%% Load all initial parameters from configuration files stored in config/
[robot_param, environment_param, gait_planning_param, control_param, equilibrium_eval_param, ani_settings, save_settings, ...
    plot_settings, gripper_param, map_param, matching_settings, sensing_camera_param] = config_simulation(config);

%%% Define a code for current set of simulations
run_cod = 'clawar_2021_statistical_analysis';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize environment %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global d_time; global Gravity; global Ez; global contact_f; global x; global y; global z;

ini_environment(environment_param);
surface_param = ini_surface(environment_param);
surface_param = ini_graspable_points(environment_param, surface_param, gripper_param, map_param, matching_settings);

%%% For the static analysis, we want to itterate the simulation with randomized
%%% base positions

base_pos_plus_metrics = []; % prepare the array to store the metrics
RMSJointTorqueArray = []; % prepare the array for convergence detection
i = 1;

base_pos_x_min =  0.00; base_pos_x_max =  0.15;
base_pos_y_min =  0.00; base_pos_y_max =  0.15;
base_pos_z_min =  0.30; base_pos_z_max =  0.30;
for base_pos_x = base_pos_x_min:0.015:base_pos_x_max
    for base_pos_y = base_pos_y_min:0.015:base_pos_y_max
        for base_pos_z = base_pos_z_min:0.015:base_pos_z_max
            
            variables_saved = [];

            %%% Initial x and y "offset" position of the base from the neutral position of the base center [m]
            robot_param.x_base_pos_offset_from_the_neutral_pos = base_pos_x;
            robot_param.y_base_pos_offset_from_the_neutral_pos = base_pos_y;
            %%% Height of base relative to map [m]
            robot_param.base_height = base_pos_z;
            %%% Base position [m] 2x1 vector. or 'default' for default setting
            robot_param.base_pos_xy = [base_pos_x;base_pos_y];

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [LP, SV, des_SV, POS_e, ORI_e, cont_POS, ani_settings] = ini_robot(robot_param, gait_planning_param, surface_param, ani_settings);
            shape_robot = vis_create_robot_model(ani_settings, LP);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize gait %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [gait_planning_param, motion_planning_param] = ini_gait(gait_planning_param, SV);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [run_id, run_date] = ini_id(robot_param, environment_param, gait_planning_param, control_param);
            video   = ini_video_file(ani_settings, run_cod, run_id, run_date);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Simulation loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            for time = 0:d_time:environment_param.time_max
            % Display time
            disp(time)

            %     if norm(SV.R0(1:2) - gait_planning_param.goal(1:2)) < 0.02
            %         break
            %     end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Gait Planning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            gait_planning_param = upd_gait_planning(gait_planning_param, surface_param, des_SV, SV, LP, POS_e, ...
                                                environment_param, robot_param, time);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Motion Planning %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [motion_planning_param, des_SV] = upd_motion_planning(motion_planning_param, gait_planning_param, ...
				                                          LP, SV, des_SV, cont_POS, POS_e, time);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Equilibrium Evaluation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            equilibrium_eval_param = upd_equilibrium_eval(equilibrium_eval_param, surface_param, ani_settings, LP, SV, POS_e);


            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot controller %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            SV = upd_control(SV, des_SV, control_param);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Kinematics and dynamics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            cont_POS = upd_collision_endeffector(LP, SV, POS_e, cont_POS);
            SV = upd_ground_reaction_forces_spring_damper(LP, SV, surface_param, POS_e, cont_POS);
            SV = upd_fwd_dynamics(environment_param, LP, SV, des_SV);
            [POS_e, Qe_deg, Q0_deg, SV] = upd_fwd_kinematics(LP, SV);

            evaluation_param = upd_manipulability(LP,SV,evaluation_param);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save selected variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            variables_saved = upd_variables_saved(variables_saved, save_settings, time, LP, SV, POS_e, environment_param, equilibrium_eval_param, evaluation_param, gait_planning_param);

            SV = upd_grasp_detach(LP, SV, des_SV, environment_param, equilibrium_eval_param, variables_saved);
            % 	SV.sup = des_SV.sup;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Climbing animation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                     % Visualization
%                     if rem(time,1/ani_settings.frame_rate) == 0 && strcmp(ani_settings.display,'on')
%                         inc = environment_param.inc;
%                         figure(1); clf; hold on;
%                         vis_robot(LP, SV, POS_e, shape_robot, inc, ani_settings);
%                         ani_settings = vis_animation_range(motion_planning_param,surface_param,ani_settings,inc);
%                         vis_surface(inc, ani_settings);
%                         vis_graspable_points(surface_param,inc,ani_settings);
%                         vis_graspable_points_in_reachable_area(gait_planning_param, inc, ani_settings);
%                         vis_reachable_area(SV, LP, gait_planning_param, inc, ani_settings, surface_param);
%                         vis_goal(gait_planning_param, inc, ani_settings);
%                         vis_support_triangle(SV,POS_e,inc,ani_settings);
%                         vis_com_projection(LP, SV, inc, ani_settings);
%                         vis_next_desired_position(gait_planning_param, inc, ani_settings);
%                         vis_stability_polyhedron(inc, equilibrium_eval_param, ani_settings);
%                         vis_vectors(inc,ani_settings,equilibrium_eval_param,LP,SV);
%                         vis_animation_settings(ani_settings,surface_param,time);
%                         vis_trajectory_4legged(environment_param,ani_settings,variables_saved);
%                         vis_sensing_camera_fov(SV, inc, sensing_camera_param, ani_settings);
%                         writeVideo(video,getframe(1));
%                     end 

            %% %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% coinvergence detection
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% if the previous 10 data every 0.02 sec is within the preset
            %%% variance, it is detected as convergent, and escape from
            %%% this for loop without waiting until time_max
            if rem(time, 0.02) == 0
                RMSJointTorqueArray = [RMSJointTorqueArray; rms(variables_saved.tau(end,:))];
            end
            if time >= 0.2
                lastRMSJointTorqueArray = RMSJointTorqueArray(size(RMSJointTorqueArray,1)-(10-1):size(RMSJointTorqueArray,1));
                meanLastRMSJointTorqueArray = mean(lastRMSJointTorqueArray);
                convergence_is_OK = true;
                % 
                threshold_to_detect_convergence = 0.5;
                for count = 1:1:10
                    if abs( RMSJointTorqueArray(size(RMSJointTorqueArray,1)-(count-1)) - meanLastRMSJointTorqueArray ) > threshold_to_detect_convergence
                        convergence_is_OK = false;
                    end
                end

                if convergence_is_OK == true
                    break
                end   
            end
                
            end
            %%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Saving variables %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            close(video);
            data = sav_data_file(variables_saved, save_settings, run_cod, run_id, run_date, config);
            %     % Plot graphs
            %     vis_plot_graph(data, plot_settings, LP, inc, surface_param, gait_planning_param);

            toc
            %% 
            %%% Extraxt the each metric: TSM, GIA margin, torque, and manipulability from the one loop result
            % 0) store the base position 
            static_analyzation_dataset.base_position_x(i,1) = data.R0(1,1);
            static_analyzation_dataset.base_position_y(i,1) = data.R0(1,2);
            static_analyzation_dataset.base_position_z(i,1) = data.R0(1,3);
            
            % maximum torque in all the joints
            static_analyzation_dataset.max_absolute_torque_of_all_joints(i,1) = max( abs(data.tau(end,:)) );
            % root mean square value of the torque
            static_analyzation_dataset.rms_torque_of_all_joints(i,1) = rms( data.tau(end,:) );
            % average of the manipularbility measure
            static_analyzation_dataset.mean_manipularbility_measure_of_four_limbs(i,1) = mean([data.manipulability1(end) data.manipulability2(end) ... 
                                   data.manipulability3(end) data.manipulability4(end)]);
            % minimum of the manipularbility measure
            static_analyzation_dataset.min_manipularbility_measure_of_four_limbs(i,1) = min([data.manipulability1(end) data.manipulability2(end) ... 
                                   data.manipulability3(end) data.manipulability4(end)]);
            % maximum of the manipularbility measure
            static_analyzation_dataset.max_manipularbility_measure_of_four_limbs(i,1) = max([data.manipulability1(end) data.manipulability2(end) ... 
                                   data.manipulability3(end) data.manipulability4(end)]);
                               
            % average of TSM
            static_analyzation_dataset.tsm(i,1) = data.tsm(end);
            % average of TSM
            static_analyzation_dataset.gia_margin(i,1) = data.gia_margin(end);
%             % finally, metrics are coupled with the base position vector
%             base_pos_plus_metrics(i,:) = [ data.R0(1,:)  max_torque  rms_torque  mean_manipularbility  mean_tsm  mean_gia_margin];
%             
            i = i + 1;
        end
    end           
end
%%
% Convert to table and save csv
% dir_name = ['dat/' run_cod '/' run_id];
dataset = struct2table(static_analyzation_dataset);
writetable(dataset,['dat/' run_cod '/' run_date '_static_analyzation_dataset.csv']);

% %% set the robot pose neutral agian before the figure visualization
% %%% Initial x and y "offset" position of the base from the neutral position of the base center [m]
% robot_param.x_base_pos_offset_from_the_neutral_pos = 0;
% robot_param.y_base_pos_offset_from_the_neutral_pos = 0;
% %%% Height of base relative to map [m]
% robot_param.base_height = ( base_pos_z_min + base_pos_z_max ) / 2;
% %%% Base position [m] 2x1 vector. or 'default' for default setting
% robot_param.base_pos_xy = [( base_pos_x_min + base_pos_x_max ) / 2; ( base_pos_y_min + base_pos_y_max ) / 2];
% [LP, SV, des_SV, POS_e, ORI_e, cont_POS, ani_settings] = ini_robot(robot_param, gait_planning_param, surface_param, ani_settings);
% shape_robot = vis_create_robot_model(ani_settings, LP);

inc = environment_param.inc;

% Rotation matrix
rot = rpy2dc([0;pi*inc/180;0])';
for j=1:size(dataset,1)
    base_positions(j,1:3) = ( rot' * [dataset.base_position_x(j,1) dataset.base_position_y(j,1) dataset.base_position_z(j,1)]' )';
end

%%
for fig_num = 1 : 1 : 5
    figure(fig_num); clf; hold on;
    vis_robot(LP, SV, POS_e, shape_robot, inc, ani_settings);
    ani_settings = vis_animation_range(motion_planning_param,surface_param,ani_settings,inc);
    vis_surface(inc, ani_settings, environment_param, surface_param);
    vis_graspable_points(surface_param,inc,ani_settings);
    vis_graspable_points_in_reachable_area(gait_planning_param, inc, ani_settings);
    vis_reachable_area(SV, LP, gait_planning_param, inc, ani_settings, surface_param);
    vis_goal(gait_planning_param, inc, ani_settings);
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
    vis_global_path(gait_planning_param, inc, ani_settings);
    vis_goal(gait_planning_param, inc, ani_settings);
    % writeVideo(video,getframe(1));

    x_for_vis = base_positions(:,1);
    y_for_vis = base_positions(:,2);
    z_for_vis = base_positions(:,3);
    % c = base_pos_plus_metrics(:,4);
    scatter3( x_for_vis, y_for_vis, z_for_vis, 50, dataset.gia_margin(:,1), 'filled', 'MarkerFaceAlpha', 0.5);

    % Axis, grid and camera angle (copied from vis_animationa_settings)
    axis equal; grid on; view(ani_settings.camera_az,ani_settings.camera_el);

    % colomap and bar
    colormap(jet);
    colorbar;

    % title and labels  (copied from vis_animationa_settings)
    title('distribution');
    xlabel('\it{x} \rm{[m]}','FontName',ani_settings.font_name,'FontSize',ani_settings.font_size);
    ylabel('\it{y} \rm{[m]}','FontName',ani_settings.font_name,'FontSize',ani_settings.font_size);
    zlabel('\it{z} \rm{[m]}','FontName',ani_settings.font_name,'FontSize',ani_settings.font_size);
    xlim(ani_settings.x_lim); ylim(ani_settings.y_lim); zlim(ani_settings.z_lim);

    data = sav_data_file(variables_saved, save_settings, run_cod, run_id, run_date, config);
    % Plot graphs
    vis_plot_graph(data, plot_settings, LP, inc, surface_param, gait_planning_param);
end

disp("Itterative calculation is done. The analysis dataset results are produced under the following folder: ")
disp("dat/")
disp(run_cod)

    
% EOF