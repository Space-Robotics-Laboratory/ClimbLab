%%%%%% Visualization
%%%%%% vis_graph_gia_margin
%%%%%%
%%%%%% Visualize time history of GIA acceleration margin
%%%%%%
%%%%%% Created 2020-07-03
%%%%%% Warley Ribeiro
%%%%%% Last update: 2022-02-12
%%%%%% Kentaro Uno
%
%
% Plot time history of acceleration margin
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         data              : Saved data table
%         plot_settings     : plot settings (struct)

function vis_graph_gia_margin(data, plot_settings)

if strcmp(plot_settings.gia_margin,'on')
    figure(plot_settings.gia_margin_fig_number)
    
    % plot TSM
    plot(data.time,data.gia_margin,'-','LineWidth',plot_settings.width);
    % figure settings
    title('GIA Acceleration Margin','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    xlabel('\itt \rm[s]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    ylabel('\it{a}_{\rmmarg} \rm[m/s^2]','FontName',plot_settings.font_name,'FontSize',plot_settings.font_size);
    set(gca,'fontsize',plot_settings.font_size,'LineWidth',plot_settings.width/2);
    set(gcf,'color','w');
    grid on;
end

end