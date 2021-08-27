%%%%%% Equilibrium
%%%%%% gia_limit
%%%%%% 
%%%%%% Obtain maximum GIA acceleration for normal directions of tumbling axes
%%%%%% 
%%%%%% Created 2020-02-04
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-08-20
%
% Calculate maximum acceleration for the normal direction n_ab
%
%                            sum{Fj.{(pb-pj)x(pa-pj)}} + M0.(pa-pb) + F0.(pbxpa)
%                gia_lim = -------------------------------------------------------
%                                           m ||(pg-pa)x(pg-pb)|| 
%
% 
% Function variables:
%
%     OUTPUT
%         gia_limit_nab       : Acceleration limit for all possible tumbling axis faces of the equilibrium polyhedron 
%                               (3 x tumbling_axes_number matrix)
%     INPUT
%        n                    : Total number of legs (scalar)
%        grasp_flag           : Flag of grasping condition of each leg (1: grasping, 0: not grasping) (1xn vector)
%        mass                 : Total mass of the robot [kg] (scalar)
%        tumbling_axes        : Matrix with the number legs for tumbling axes (matrix: tumbling_axes_number x 2). Each 
%                               row represents one tumbling axis, while the columns represent the number of the leg for 
%                               that specific axis
%        tumbling_axes_number : Total number of possible tumbling axis (scalar)
%        POS_e                : End-effector positions POS_e = [p1 p2 ... pn] [m] (3xn matrix) 
%        M0                   : External moment acting at the center of gravity [Nm] (3x1 vector)
%        F0                   : External force acting at the center of gravity [N] (3x1 vector)
%        F_hold               : Maximum holding force [N] (scalar)
%        n_ab                 : Normal vector to the tumbling axis from CoG for all possible tumbling axes (3 x tumbling_axes_number matrix)
%        n_ab_u               : Unitary normal vector to the tumbling axis from CoG for all possible tumbling axes (3 x tumbling_axes_number matrix)

function gia_limit_nab = gia_limit(n, mass, grasp_flag, tumbling_axes, tumbling_axes_number, POS_e, M0, F0, F_hold, n_ab, n_ab_u)

% Initialize variables
gia_limit_nab = zeros(3,tumbling_axes_number);

if tumbling_axes_number == 0
    gia_limit_nab(1:3) = zeros(3,1);
else

for i = 1:tumbling_axes_number
    a = tumbling_axes(i,1); b = tumbling_axes(i,2);
    % Tumbling axis initial and final points
    pa = POS_e(:,a);
    pb = POS_e(:,b);
    
    % Moment due to external force/moment
    Mab = M0'*(pa-pb) + F0'*cross(pb,pa);
	% Moment due to holding force
    for j = 1:n
        if j ~= a && j ~= b && grasp_flag(j) == 1
            pj = POS_e(:,j);
            Mab = Mab + F_hold*[0 0 -1]*cross((pb-pj),(pa-pj));
        end
    end
    % Maximum GIA
    gia_limit_nab(:,i) = Mab/(mass*norm(n_ab(:,i)))*n_ab_u(:,i);
    
end

end

end