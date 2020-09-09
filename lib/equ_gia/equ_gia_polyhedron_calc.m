%%%%%% Equilibrium
%%%%%% equ_gia_polyhedron_calc
%%%%%% 
%%%%%% Calculate Gravito-Inertial Acceleration Stability Polyhedron
%%%%%% 
%%%%%% Created 2020-01-20
%%%%%% by Warley Ribeiro
%%%%%% Last update: 2020-06-15
%
% Considering the tumble stability with the addition of gripping forces to prevent the tumbling motion, the following
% equation describe the limit condition for one tumbling axis
%
%             m.a_gi.{(pg-pa)x(pg-pb)} = sum{Fj.{(pb-pj)x(pa-pj)}} + M0.(pa-pb) + F0.(pbxpa)
%
% If (g-a) is considered as a variable (the Gravito-inertial acceleration), the equation for the limit of the equilibrium 
% relative to a tumbling axis is a plane in the three-dimensional cartesian space. The intersection of planes for all the 
% possible tumbling axes gives a convex polyhedron. This is the stability polyhedron for the gravito-inertial acceleration, 
% which can used to define if a robot is in dynamic equilibrium or not, if the GIA vector (g-a) is inside the polyhedron.
%
%         m   : Total mass of the robot
%         a_gi: Gravito-inertial acceleration
%         pa  : Position of the first point of the tumbling axis
%         pb  : Position of the second point of the tumbling axis
%         pg  : Position of the center of gravity
%         pj  : Position of the other grasping points
%         Fj  : Maximum holding force for the position j
%         M0  : External moment applied to the center of gravity
%         F0  : External force applied to the center of gravity
%
%
% Function variables:
%
%     OUTPUT
%        polyhedron   : Variables to define the shape of the polyhedron (struct)
%         polyhedron.plane_point     : Maximum accelerations in a normal direction of tumbling axis [m/s^2] (3xn matrix)
%         polyhedron.plane_vector    : Normal vector to tumbling axis from center of gravity (3xn matrix)
%         polyhedron.plane_point_exp : Point in the limit plane based on robot position and the expansion factor (3xn matrix)
%         polyhedron.edge_vec        : Direction vector for the edges of the polyhedron [m] (3xn matrix)
%         polyhedron.edge_point      : Position of a point in the line of the edge where the z position is null (3xn matrix)
%         polyhedron.vertex          : Position of the corners of the polyhedron (3x(n+1) matrix)
%        gia              : Gravito-Inertial Acceleration vector [m/s^2] (3x1 vector)
%        equ_flag         : Flag of equilibrium condition (1: equilibrium, 0: not in equilibrium) (scalar)
%    INPUT
%        POS_e            : End-effector positions POS_e = [p1 p2 ... pn] [m] (3xn matrix) 
%        pg               : Center of Gravity position [m] (3x1 vector)
%        a_g              : Acceleration of the center of gravity [m/s^2] (3x1 vector)
%        mass             : Total mass of the robot [kg] (scalar)
%        grasp_flag       : Flag of grasping condition of each leg (1: grasping, 0: not grasping) (1xn vector)
%        F_hold           : Maximum holding force [N] (scalar)
%        F0               : External force acting at the center of gravity [N] (3x1 vector)
%        M0               : External moment acting at the center of gravity [Nm] (3x1 vector)
%        plot_on          : Variable to plot polyhedron (1: plot, 2: do not plot) (logic)
%        floor_base       : Vertical (z-axis) coordinate of floor for polyhedron base [m] (scalar)
%        expansion_factor : Expansion factor for the acceleration vector compared to the position vectors (scalar)

function [polyhedron, gia, equ_flag] = equ_gia_polyhedron_calc(POS_e, pg, a_g, mass, grasp_flag, F_hold, F0, M0, plot_on, ...
                                                               floor_base, expansion_factor)

global Gravity

n = size(POS_e,2);
% Determine tumbling axes a-b
[tumbling_axes, tumbling_axes_number] = equ_tumbling_axes_ab(n, grasp_flag);
% Calculate normal vector
[n_ab, n_ab_u] = normal_vector_nab(tumbling_axes, tumbling_axes_number, POS_e, pg);

% Calculate maximum acceleration for each normal direction
gia_limit_nab = gia_limit(n, mass, grasp_flag, tumbling_axes, tumbling_axes_number, POS_e, M0, F0, F_hold, n_ab, n_ab_u);

% Gravito-Inertial Acceleration (g-a)
gia = Gravity - a_g;

% Check equilibrium based on the current GIA and maximum acceleration limit
equ_flag = 1;
for i = 1:tumbling_axes_number
    if gia'*n_ab_u(:,i) > norm(gia_limit_nab(:,i))
        % Equilibrium flag
        equ_flag = 0;
    end
end

% Polyhedron variables
polyhedron.plane_point = gia_limit_nab;
polyhedron.plane_vector = n_ab;

if plot_on 
    % Shrink vector and move to center of gravity
    polyhedron.plane_point_exp = expansion_factor*polyhedron.plane_point + pg;
    % Calculate intersection lines
    for i = 1:tumbling_axes_number
        if i == tumbling_axes_number
            j = 1;
        else
            j = i+1;
        end
        polyhedron.edge_vec(:,i) = cross(n_ab_u(:,i),n_ab_u(:,j));
        if abs(polyhedron.edge_vec(3,i)) > 0.0001
            polyhedron.edge_point(3,i) = 0;
            A = [n_ab_u(1,i) n_ab_u(2,i);
                 n_ab_u(1,j) n_ab_u(2,j)];
            B = [n_ab_u(:,i)'*polyhedron.plane_point_exp(:,i);
                 n_ab_u(:,j)'*polyhedron.plane_point_exp(:,j)];
            polyhedron.edge_point(1:2,i) = A\B;
        end
    end
    
    % Calculate intersection points
    for i = 1:tumbling_axes_number
        % Lines and floor base
        ind = (floor_base - polyhedron.edge_point(3,i))/polyhedron.edge_vec(3,i);
        polyhedron.vertex(:,i) = polyhedron.edge_point(:,i) + ind*polyhedron.edge_vec(:,i);
    end
    % Between lines
    A = [polyhedron.edge_vec(:,1) -polyhedron.edge_vec(:,2)];
    B = polyhedron.edge_point(:,1) - polyhedron.edge_point(:,2);
    ind = pinv(A)*B;
    polyhedron.vertex(:,end+1) = polyhedron.edge_point(:,1) - ind(1)*polyhedron.edge_vec(:,1);
end

end

% EOF