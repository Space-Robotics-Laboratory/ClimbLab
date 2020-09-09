%%%%%% Visualization
%%%%%% vis_graph_inclination_margin
%%%%%%
%%%%%% Visualize time history of inclination margin
%%%%%%
%%%%%% Created 2020-07-03
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-07-03
%
%
% Plot time history of inclination margin
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         data              : Saved data table
%         plot_settings     : plot settings (struct)

function vis_graph_inclination_margin(data, plot_settings)

if strcmp(plot_settings.inclination_margin,'on')
    figure(plot_settings.inclination_margin_fig_number)
    
    % plot TSM
    plot(data.time,data.inclination_margin,'-','LineWidth',plot_settings.width);
    % figure settings
    title('GIA Inclination Margin','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    xlabel('\itt \rm[s]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    ylabel('\it{inc}_{\rmmarg} \rm[deg]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    set(gca,'fontsize',plot_settings.font_size,'LineWidth',plot_settings.width/2);
    grid on;
end

end