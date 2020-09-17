%%%%%% Equilibrium
%%%%%% equ_gia_acceleration_margin
%%%%%% 
%%%%%% Calculate acceleration margin based on the stability polyhedron
%%%%%% 
%%%%%% Created 2020-02-05
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-06-16
%
% Considering the tumble stability with the addition of gripping forces to prevent the tumbling motion, the following
% equation describe the limit condition for one tumbling axis
%
%             m.a_gi.{(pg-pa)x(pg-pb)} = sum{Fj.{(pb-pj)x(pa-pj)}} + M0.(pa-pb) + F0.(pbxpa)
%
% The acceleration margin is the following, considering all tumbling axes
%
%                                                    a_gi.{(pg-pa)x(pg-pb)}
%                    acc_marg =  ||a_gi_lim||  -  ----------------------------
%                                                      ||(pg-pa)x(pg-pb)||
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
%         acc_margin       : Acceleration margin considering the stability polyhedron [m/s^2] (scalar)
%         acc_margin_ab    : Acceleration margin for each tumbling axis [m/s^2] (1xn vector)
%     INPUT
%         polyhedron       : Variables to define the shape of the polyhedron (struct)
%         gia              : Gravito-Inertial Acceleration [m/s^2] (3x1 vector)
%         equ_flag         : Flag of equilibrium condition (1: equilibrium, 0: not in equilibrium) (scalar)


function [acc_margin, acc_margin_ab] = equ_gia_acceleration_margin(polyhedron, gia, equ_flag)

% Number of tumbling axes
tumbling_axes_number = size(polyhedron.plane_vector,2);
% Initialize variable
acc_margin_ab = zeros(1,tumbling_axes_number);

if equ_flag == 0
    % If not in equilibrium, margin is zero
    acc_margin = 0;
else
    for i = 1:tumbling_axes_number
        % Margin for i-th plane
        acc_margin_ab(i) = norm(polyhedron.plane_point(:,i)) - gia'*polyhedron.plane_vector(:,i)/norm(polyhedron.plane_vector(:,i));
    end
    acc_margin = min(acc_margin_ab);
end

end

% EOF