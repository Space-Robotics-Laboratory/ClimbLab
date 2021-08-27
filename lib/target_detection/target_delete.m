%%%%%% Target Delete
%%%%%% target_delete.m
%%%%%% 
%%%%%% Select the targets located above the threshold in z-coordinate
%%%%%% 
%%%%%% Created 2021-04-16
%%%%%% Keigo Haji
%%%%%% Last update: 2021-04-16
%%%%%% Keigo Haji
% 
% Delete the targets whose z positions are less than threshold
% 
% 
% Function variables:
% 
% OUTPUT
%   graspable_points   : 3*N matrix contains position vectors of graspable points [m] (x;y;z)
% 
% INPUT
%   graspable_points   : 3*M(>N) matrix contains position vectors of graspable points [m] (x;y;z)
%   matching_settings : settings for matching
%
% 
function [graspable_points] = target_delete(graspable_points,low_threshold)

% Check the number of graspable points
[~,m] = size(graspable_points);

% Replace the positions of targets located under the threshold to NaN values
for i=1:m
    if graspable_points(3,i) < low_threshold
       graspable_points(:,i) = [NaN; NaN; NaN]; 
    else
    end
end

% Delete NaN columns
graspable_points = rmmissing(graspable_points, 2);

end

