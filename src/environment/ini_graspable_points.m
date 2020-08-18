%%%%%% Initialize
%%%%%% ini_graspable_points
%%%%%% 
%%%%%% Extract graspable points from mesh
%%%%%% 
%%%%%% Created 2020-05-12
%%%%%% Yusuke Koizumi
%%%%%% Last update: 2020-07-11 by Kentaro Uno
%
%
% Extract graspable points according to the graspable_points_detection_type
%
% Function variables:
%
%     OUTPUT
%         surface_param.graspable_points   : 3xN matrix contains position vectors of graspable points [m] (x;y;z)
%     INPUT
%         graspable_points_detection_type        : Graspable points detection type (char: 'all','peaks', etc. or int: 0~100 [%]) 
%     USGAE EXAMPLE
%         graspable_points_detection_type = 50;
%         surface_param = ini_graspable_points(graspable_points_detection_type, surface_param);


function surface_param = ini_graspable_points(environment_param, surface_param)
global x;global y;global z;

[n,m]=size(z);
surface_param.graspable_points=zeros(3,n*m);
[X,Y] = meshgrid(x,y);
surface_param.graspable_points(1,:)=reshape(X,1,n*m);
surface_param.graspable_points(2,:)=reshape(Y,1,n*m);
surface_param.graspable_points(3,:)=reshape(z,1,n*m);

% set the floor level variable to have a z directional height in global
% frame.
surface_param.floor_level = mean(surface_param.graspable_points(3,:));

switch environment_param.graspable_points_detection_type
    case 'all'
        ;
    case 'peaks'
        % will be added in the next update 
        % need some func. to detect peaks
        ;
    otherwise
        if isnumeric(environment_param.graspable_points_detection_type)
            if environment_param.graspable_points_detection_type >=0 && environment_param.graspable_points_detection_type <=100
                cnt = uint64( n * m * environment_param.graspable_points_detection_type/100 );
                while cnt > 0 && ~isempty(surface_param.graspable_points)
                    rm = randi(size(surface_param.graspable_points,2)) ;
                    surface_param.graspable_points(:,rm) = []; % remove the point from surface_param.graspable_points.
                    cnt = cnt - 1;
                end
            end
        end
end
end