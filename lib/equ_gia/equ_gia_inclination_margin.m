%%%%%% Equilibrium
%%%%%% equ_gia_inclination_margin
%%%%%% 
%%%%%% Calculate inclination margin based on the stability polyhedron
%%%%%% 
%%%%%% Created 2020-02-06
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-06-16
%
% Considering the tumble stability with the addition of gripping forces to prevent the tumbling motion, the following
% equation describe the limit condition for one tumbling axis
%
%             m.a_gi.{(pg-pa)x(pg-pb)} = sum{Fj.{(pb-pj)x(pa-pj)}} + M0.(pa-pb) + F0.(pbxpa)
%
% The angular margin for the total acceleration is the following, considering all tumbling axes
%
%                                       (pg-pa)x(pg-pb).a_gi               ||(a_gi_lim||
%              inc_marg = min(acos(-------------------------------) - acos(-------------))
%                                    ||(pg-pa)x(pg-pb)|| ||a_gi||            ||a_gi||
%
%
%         a_gi: Gravito-inertial acceleration
%         pa  : Position of the first point of the tumbling axis
%         pb  : Position of the second point of the tumbling axis
%         pg  : Position of the center of gravity
%
% Function variables:
%
%     OUTPUT
%         inclination_margin    : Inclination margin for total acceleration considering the support polyhedron [deg] (scalar)
%         inclination_margin_ab : Inclination margin for each tumbling axis [rad] (1xn vector)
%     INPUT
%         polyhedron            : Variables to define the shape of the polyhedron (struct)
%         gia                   : Gravito-Inertial Acceleration [m/s^2] (3x1 vector)
%         equ_flag              : Flag of equilibrium condition (1: equilibrium, 0: not in equilibrium) (scalar)


function [inclination_margin, inclination_margin_ab] = equ_gia_inclination_margin(polyhedron, gia, equ_flag)

% Number of tumbling axes
tumbling_axes_number = size(polyhedron.plane_vector,2);
% Initialize variable
inclination_margin_ab = zeros(1,tumbling_axes_number);

if equ_flag == 0
    % If not in equilibrium, margin is zero
    inclination_margin = 0;
else
    for i = 1:tumbling_axes_number
        if norm(gia) < norm(polyhedron.plane_point(:,i))
            % If acceleration is smaller than limit, inclination does not affect margin
            inclination_margin_ab(i) = pi;
        else
            % Margin for i-th plane
            inclination_margin_ab(i) = acos(polyhedron.plane_vector(:,i)'*gia/(norm(polyhedron.plane_vector(:,i))*norm(gia))) ...
                                   - acos(norm(polyhedron.plane_point(:,i))/norm(gia));
        end
    end
    inclination_margin = min(inclination_margin_ab)*180/pi;
end
end

% EOF