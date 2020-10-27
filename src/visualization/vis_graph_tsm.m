%%%%%% Visualization
%%%%%% vis_graph_tsm
%%%%%%
%%%%%% Visualize time history of TSM
%%%%%%
%%%%%% Created 2020-07-03
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-27
%
%
% Plot time history of TSM 
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         data              : Saved data table
%         plot_settings     : plot settings (struct)

function vis_graph_tsm(data, plot_settings)

if strcmp(plot_settings.tsm,'on')
    figure(plot_settings.tsm_fig_number)
    
    % plot TSM
    plot(data.time,data.tsm,'-','LineWidth',plot_settings.width);
    % figure settings
    title('Tumble Stability Margin','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    xlabel('\itt \rm[s]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    ylabel('\it{TSM} \rm[m]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    set(gca,'fontsize',plot_settings.font_size,'LineWidth',plot_settings.width/2);
    set(gcf,'color','w');
    grid on;
end

end