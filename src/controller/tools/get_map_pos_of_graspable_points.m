%%%%%% Calculate
%%%%%% get_map_pos_of_graspable_points
%%%%%% 
%%%%%% Obtain the correspondent graspable points' positions (including height) for a x-y set
%%%%%% 
%%%%%% Created 2021-03-09
%%%%%% Keigo Haji
%%%%%% Last update: 2021-03-09
%&%%%% Keigo Haji
%
%
% For a given set of points, obtain the closest graspable points in the map
%
% Function variables:
%
%     OUTPUT
%         xm           : x-coordinate position for the map (1xn, where n is the number of points to be checked)
%         ym           : y-coordinate position for the map (1xn)
%         zm           : z-coordinate position for the map (1xn)
%     INPUT
%         xp           : Given x-coordinate position of the point(s) to be checked (1xn)
%         yp           : Given y-coordinate position of the point(s) to be checked (1xn)
%         surface_param         : Parameters for surface (class)
%           surface_param.graspable_points : 3xN matrix contains position vectors of graspable points [m] (x;y;z)

function [xm,ym,zm] = get_map_pos_of_graspable_points(xp,yp,surface_param)


% number of points to be checked
n = length(xp);

    for i = 1:n
        % get the graspable point of the map nearest point 
        GPs = surface_param.graspable_points;
        [~,id_GP] = min((GPs(1,:) - xp(i)).^2 + (GPs(2,:) - yp(i) ).^2);
        % get the height corresponding to that point      
        xm(i) = GPs(1,id_GP);
        ym(i) = GPs(2,id_GP);
        zm(i) = GPs(3,id_GP);
    end
    
end
