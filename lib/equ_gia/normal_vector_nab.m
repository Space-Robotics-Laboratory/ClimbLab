%%%%%% Equilibrium
%%%%%% normal_vector_nab
%%%%%% 
%%%%%% Obtain normal vectors for equilibrium polyhedron
%%%%%% 
%%%%%% Created 2020-02-03
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-02-03
%
% Calculate vector normal to tumbling axes from center of gravity, which is the normal vector to the limit planes
% 
% Function variables:
%
%     OUTPUT
%         n_ab         : Normal vector to the tumbling axis from CoG for all possible tumbling axes (3 x tumbling_axes_number matrix)
%         n_ab_u       : Unitary normal vector to the tumbling axis from CoG for all possible tumbling axes (3 x tumbling_axes_number matrix)
%     INPUT
%         tumbling_axes           : Matrix with the number legs for tumbling axes (matrix: tumbling_axes_number x 2). Each 
%                                   row represents one tumbling axis, while the columns represent the number of the leg for 
%                                   that specific axis
%         tumbling_axes_number    : Total number of possible tumbling axis (scalar)
%         POS_e                   : End-effector positions POS_e = [p1 p2 ... pn] [m] (matrix: 3 x n) 
%         pg                      : Center of Gravity position [m] (vector: 3 x 1)

function [n_ab, n_ab_u] = normal_vector_nab(tumbling_axes, tumbling_axes_number, POS_e, pg)

% Initialize variables
n_ab = zeros(3,tumbling_axes_number);
n_ab_u = n_ab;
% Calculate normal for each tumbling axis
for i = 1:tumbling_axes_number
    a = tumbling_axes(i,1); b = tumbling_axes(i,2);
    % Tumbling axis initial and final points
    pa = POS_e(:,a);
    pb = POS_e(:,b);
    % Normal vector
    n_ab(:,i) = cross((pg-pa),(pg-pb));
    % Normal unit vector
    n_ab_u(:,i) = n_ab(:,i)/norm(n_ab(:,i));
end
