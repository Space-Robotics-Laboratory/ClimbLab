%%%%%% Calculate
%%%%%% get_map_pos
%%%%%% 
%%%%%% Obtain the correspondent map position (including height) for a x-y set
%%%%%% 
%%%%%% Created 2019-09-30
%%%%%% Victoria Keo, Warley Ribeiro
%%%%%% Last update: 2019-12-06
%
%
% For a given set of points xp, obtain the closest points in the map regardless of the map's resolution, including the map
% height for those points
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

function [xm,ym,zm] = get_map_pos(xp,yp)


global x ; global y ; global z

% number of points to be checked
n = length(xp);

    for i = 1:n
        % get the (x,y) coordinates of the map nearest point 
        [ ~, ix ] = min(abs( x-xp(i) )); xm(i) = x(ix); 
        [ ~, iy ] = min(abs( y-yp(i) )); ym(i) = y(iy); 
        % get the height corresponding to that point 
        zm(i) = z(iy,ix); % z(y,x)!!                    
    end
    
end
