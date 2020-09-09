%%%%%% Visualization
%%%%%% vis_footholds
%%%%%%
%%%%%% Visualize time history of footholds in a 2D map
%%%%%%
%%%%%% Created 2020-06-01
%%%%%% Kentaro Uno
%%%%%% Last update: 2020-06-02
%
%
% Plot time history of footholds 
%
% Function variables:
%
%     OUTPUT
%         -
%     INPUT
%         LP                                : Link Parameters (SpaceDyn class)
%         plot_settings.footholds           : on/off (String)
%         plot_settings.footholds_fig_number: Figure number (integer)
%         inc                               : Surface inclination [deg] (scalar)
%         surface_param.graspable_points    : Position of graspable points
%         path_planning_param.footholds_history_limb                   
%             : Three-dimensional matrix to store footholds of one limb (double), which is updated in upd_swing_next_pos_crawl_fixed_stride()
%               e.g.) path_planning_param.footholds_history_limb(1,3,2) means: x(1) position of the third foothold of limb 2


function vis_footholds(LP, plot_settings, inc, surface_param, path_planning_param)
if strcmp(plot_settings.footholds,'on')
    figure(plot_settings.footholds_fig_number)
    
    % plot the graspabe points first
    plot(surface_param.graspable_points(1,:),surface_param.graspable_points(2,:),'xb')
    hold on
    surf_AA = rpy2dc([0;pi*inc/180;0])';
    % plot(path_planning_param.com_projection_history(1,:), path_planning_param.com_projection_history(2,:),'-k','LineWidth',2)

    % color settings
    hue_for_limb_1 = 0.80; % magenta
    hue_for_limb_2 = 0.6;  % blue
    hue_for_limb_3 = 1.0;  % red
    hue_for_limb_4 = 0.35; % green

    % plot footholds history
    for i=1:LP.num_limb
        footholds = [];
        footholds = path_planning_param.footholds_history_limb(:,:,i);
        if path_planning_param.footholds_history_limb(:,size(path_planning_param.footholds_history_limb,2),i) == [0;0;0]
            footholds(:,size(path_planning_param.footholds_history_limb,2)) = [];
        end
        plot(footholds(1,:),footholds(2,:),':k','LineWidth',2)
        
        hsv = zeros(size(footholds,2), 3);
        switch i
            case 1 % limb 1
                hue = hue_for_limb_1;
            case 2 % limb 2 
                hue = hue_for_limb_2;
            case 3 % limb 3 
                hue = hue_for_limb_3;
            case 4 % limb 4 
                hue = hue_for_limb_4;
        end
                
        for j=1:size(hsv, 1)
            hsv(j, :) = [hue (1/size(hsv, 1)*j) 0.95];
            rgb = hsv2rgb(hsv);
            plot(footholds(1,j),footholds(2,j),'o','MarkerEdgeColor',rgb(j, :),...
                                                   'MarkerFaceColor',rgb(j, :),'MarkerSize',8)
        end
        
        % TODO: first position can be non-colored by plotting independently
        % plot(pos_init(1,i),pos_init(2,i),'ok','LineWidth',1,'MarkerSize',8)
        
        hold on
    end
    
    % plot the CoM projection history (to be added)
    
    % figure settings
    title('Footholds history');
    xlabel('\itx_g \rm[m]','FontName','calibri','FontSize',10); ylabel('\ity_g \rm[m]','FontName','calibri','FontSize',10);
    set(gca,'fontsize',10);
    ax = gca;
    ax.XLim = [min(surface_param.graspable_points(1,:)) max(surface_param.graspable_points(1,:))]; ax.YLim = [min(surface_param.graspable_points(2,:)) max(surface_param.graspable_points(2,:))];
    ax.DataAspectRatio = [1 1 1];
    grid on;
end
% % save as fig file (to be added)
% if sav == 1
%     saveas(gcf, [dir_name '/' cod 'footholds_history' '.fig'], 'fig')
% end
end