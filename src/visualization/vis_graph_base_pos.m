%%%%%% Visualization
%%%%%% vis_graph_base_pos
%%%%%%
%%%%%% Visualize time history of base position
%%%%%%
%%%%%% Created 2020-07-03
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-03
%
%
% Plot time history of base position
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         data              : Saved data table
%         plot_settings     : plot settings (struct)

function vis_graph_base_pos(data, plot_settings)

if strcmp(plot_settings.base_pos,'on')
    figure(plot_settings.base_pos_fig_number)
    
    % plot TSM
    plot(data.time,data.R0,'-','LineWidth',plot_settings.width);
    % figure settings
    title('Base position','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    xlabel('\itt \rm[s]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    ylabel('\it{x}_{\rmb} \rm[m]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    set(gca,'fontsize',plot_settings.font_size,'LineWidth',plot_settings.width/2);
    grid on;
end

end