%%%%%% Visualization
%%%%%% vis_plot_graph
%%%%%%
%%%%%% Visualize graphs
%%%%%%
%%%%%% Created 2020-07-27
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-09-08
%
%
% Plot graphs
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         data                  : Saved data table
%         plot_settings         : plot settings (struct)
%         LP                    : Link Parameters (SpaceDyn class)
%         surface_param         : Parameters for surface (struct)
%         path_planning_param   : Parameters for path planning (struct)


function vis_plot_graph(data, plot_settings, LP, inc, surface_param, path_planning_param)

% Plot time history of base position
if strcmp(plot_settings.base_pos,'on')
    R0 = data.R0;
    fig_number = plot_settings.base_pos_fig_number;
    vis_graph_time_history(data, plot_settings, R0, fig_number, 'Base position', '\it{x}_{\rmb} \rm[m]')
end
% Plot time history of base orientation
if strcmp(plot_settings.base_ori,'on')
    Q0 = data.Q0*180/pi;
    fig_number = plot_settings.base_ori_fig_number;
    vis_graph_time_history(data, plot_settings, Q0, fig_number, 'Base orientation', '\it{Q}_{\rmb} \rm[deg]')
end
% Plot time history of base velocity
if strcmp(plot_settings.base_vel,'on')
    v0 = data.v0;
    fig_number = plot_settings.base_vel_fig_number;
    vis_graph_time_history(data, plot_settings, v0, fig_number, 'Base velocity', '\it{v}_{\rmb} \rm[m/s]')
end
% Plot time history of base angular velocity
if strcmp(plot_settings.base_angvel,'on')
    w0 = data.w0*180/pi;
    fig_number = plot_settings.base_angvel_fig_number;
    vis_graph_time_history(data, plot_settings, w0, fig_number, 'Base angular velocity', '\it{w}_{\rmb} \rm[deg/s]')
end
% Plot time history of base velocity
if strcmp(plot_settings.base_acc,'on')
    vd0 = data.vd0;
    fig_number = plot_settings.base_acc_fig_number;
    vis_graph_time_history(data, plot_settings, vd0, fig_number, 'Base acceleration', '\it{a}_{\rmb} \rm[m/s^2]')
end
% Plot time history of base angular velocity
if strcmp(plot_settings.base_angacc,'on')
    wd0 = data.wd0*180/pi;
    fig_number = plot_settings.base_angacc_fig_number;
    vis_graph_time_history(data, plot_settings, wd0, fig_number, 'Base angular acceleration', '\it{wd}_{\rmb} \rm[deg/s^2]')
end


% Plot time history of joints angular position
if strcmp(plot_settings.joint_pos,'on')
    q = data.q*180/pi;
    fig_number = plot_settings.joint_pos_fig_number;
    vis_graph_time_history(data, plot_settings, q, fig_number, 'Joints position', '\it{q} \rm[deg]')
end
% Plot time history of joints angular velocity
if strcmp(plot_settings.joint_vel,'on')
    qd = data.qd*180/pi;
    fig_number = plot_settings.joint_vel_fig_number;
    vis_graph_time_history(data, plot_settings, qd, fig_number, 'Joints velocity', '\it{qd} \rm[deg/s]')
end
% Plot time history of joints angular acceleration
if strcmp(plot_settings.joint_acc,'on')
    qdd = data.qdd*180/pi;
    fig_number = plot_settings.joint_acc_fig_number;
    vis_graph_time_history(data, plot_settings, qdd, fig_number, 'Joints acceleration', '\it{qdd} \rm[deg/s^2]')
end
% Plot time history of joints torque
if strcmp(plot_settings.joint_torque,'on')
    tau = data.tau;
    fig_number = plot_settings.joint_torque_fig_number;
    vis_graph_time_history(data, plot_settings, tau, fig_number, 'Joints torque', '\it{tau} \rm[Nm]')
end


% Plot time history of legs position
if strcmp(plot_settings.leg_pos,'on')
    for i = 1:4
        pos_e = data.(genvarname(['pos_e' num2str(i)]));
        fig_number = plot_settings.leg_pos_fig_number + (i-1);
        vis_graph_time_history(data, plot_settings, pos_e, fig_number, ['Leg ' num2str(i) ' Position'], ...
                                                            ['\it{p}_' num2str(i) ' \rm[m]']);
    end
end

% Plot time history of reaction forces
if strcmp(plot_settings.reaction_force,'on')
    for i = 1:4
        pos_e = data.(genvarname(['Fe' num2str(i)]));
        fig_number = plot_settings.reaction_force_fig_number + (i-1);
        vis_graph_time_history(data, plot_settings, pos_e, fig_number, ['Leg ' num2str(i) ' Ground Reaction Force'], ...
                                                            ['\it{F}_e_' num2str(i) ' \rm[N]']);
    end
end

% Plot time history of reaction forces
if strcmp(plot_settings.reaction_force,'on')
    for i = 1:4
        pos_e = data.(genvarname(['Fe_res' num2str(i)]));
        fig_number = plot_settings.reaction_force_fig_number + 4 + (i-1);
        vis_graph_time_history(data, plot_settings, pos_e, fig_number, ['Leg ' num2str(i) ' Resultant Force'], ...
                                                            ['\it{F}_r_e_s_' num2str(i) ' \rm[N]']);
    end
end

% Plot time history of footholds 
vis_footholds(LP, plot_settings, inc, surface_param, path_planning_param);


% Plot TSM (requires saving CSV and equilibrium method as 'tsm')
vis_graph_tsm(data, plot_settings);
% Plot GIA acceleration margin (requires saving CSV and equilibrium method as 'gia')
vis_graph_acc_margin(data, plot_settings);
% Plot GIA inclination margin (requires saving CSV and equilibrium method as 'gia')
vis_graph_inclination_margin(data, plot_settings);


end