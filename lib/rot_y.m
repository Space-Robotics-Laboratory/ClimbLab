function [ Ry ] = rot_y( t )
Ry = [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
end

