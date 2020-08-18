%%%%%% Equilibrium
%%%%%% equ_tumbling_axes_ab
%%%%%% 
%%%%%% Obtain numbering of all possible tumbling axes
%%%%%% 
%%%%%% Created 2020-02-03
%%%%%% by Warley Ribeiro
%%%%%% Last update: 2020-06-10
%
%
% Obtain all possible tumbling axes numbering, accordingly to the number of the supporting legs. This function requires that
% the numbering order of legs follows either a clockwise or counter-clockwise sequence.
%
%
% Function variables:
%
%     OUTPUT
%         tumbling_axes           : Matrix with the number legs for tumbling axes (matrix: tumbling_axes_number x 2). Each 
%                                   row represents one tumbling axis, while the columns represent the number of the leg for 
%                                   that specific axis
%         tumbling_axes_number    : Total number of possible tumbling axis (scalar)
%     INPUT
%         n                       : Total number of legs (scalar)
%         grasp_flag              : Flag of grasping condition of each leg (1: grasping, 0: not grasping) (1xn vector)

function [tumbling_axes, tumbling_axes_number] = equ_tumbling_axes_ab(n, grasp_flag)

cnt = 1;
for a = 1:n
    if grasp_flag(a) == 1
        tumbling_axes(cnt,1) = a;
        for b = a+1:n
            if grasp_flag(b) == 1
                tumbling_axes(cnt,2) = b;
                cnt = cnt + 1;
                break
            end
        end
    end
end

% Close polygon with initial support point
tumbling_axes(cnt,2) = tumbling_axes(1,1);
% Total number of tumbling axes
tumbling_axes_number = cnt;
