%%%%%% Visualization
%%%%%% vis_graph_time_history
%%%%%%
%%%%%% Visualize time history
%%%%%%
%%%%%% Created 2020-07-28
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-28
%
%
% Plot time history graph
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         data              : Saved data table
%         plot_settings     : plot settings (struct)
%         y                 : data to be plotted (vector)
%         fig_number        : number for the MATLAB figure (scalar)
%         title_str         : title for the graph (string)
%         y_label           : label for y-axis (string)

function vis_graph_time_history(data, plot_settings, y, fig_number, title_str, y_label)

    figure(fig_number)
    
    % plot
    plot(data.time,y,'-','LineWidth',plot_settings.width);
    % figure settings
    title(title_str,'FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    xlabel('\itt \rm[s]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    ylabel(y_label,'FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    set(gca,'fontsize',plot_settings.font_size,'LineWidth',plot_settings.width/2);
    set(gcf,'color','w');
    grid on;


end