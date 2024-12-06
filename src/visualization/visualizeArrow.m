%%%%%% Visualization
%%%%%% visualizeArrow
%%%%%%
%%%%%% Draw an arrow
%%%%%%
%%%%%% Created:      2021.09.22 by Masazumi Imai
%%%%%% Last updated: 2021.09.30 by Masazumi Imai
%
% Draw a three-dimensional arrow from a specific origin point as patch object (cylinder + cone)
%
% Note: This function is created and rewritten for climblab based on the following function.
% ------------------------------------------------------------------------
% Georg Stillfried (2021). mArrow3.m - easy-to-use 3D arrow
% (https://www.mathworks.com/matlabcentral/fileexchange/25372-marrow3-m-easy-to-use-3d-arrow),
% MATLAB Central File Exchange. Retrieved September 24, 2021.
% ------------------------------------------------------------------------
%
% Function variables:
%   OUTPUT
%     -
%   INPUT
%     start_point : Starting point of the vector (3x1 vector)
%     end_point   : End point of the vector (3x1 vector)
%     varargin    : Patch object properties ('PropertyName',PropertyValue)

function arrow = visualizeArrow(start_point, end_point, varargin)
% mArrow3 - plot a 3D arrow as patch object (cylinder+cone)
%
% syntax:   h = mArrow3(p1,p2)
%           h = mArrow3(p1,p2,'propertyName',propertyValue,...)
%
% with:     p1:         starting point
%           p2:         end point
%           properties: 'color':      color according to MATLAB specification
%                                     (see MATLAB help item 'ColorSpec')
%                       'stemWidth':  width of the line
%                       'tipWidth':   width of the cone
%
%           Additionally, you can specify any patch object properties. (For
%           example, you can make the arrow semitransparent by using
%           'facealpha'.)
%
% example1: h = mArrow3([0 0 0],[1 1 1])
%           (Draws an arrow from [0 0 0] to [1 1 1] with default properties.)
%
% example2: h = mArrow3([0 0 0],[1 1 1],'color','red','stemWidth',0.02,'facealpha',0.5)
%           (Draws a red semitransparent arrow with a stem width of 0.02 units.)
%
% hint:     use light to achieve 3D impression
%

%%% Property specification setteing
property_name = {'EdgeColor'};
property_value = {'none'};
% Evaluate property specifications
for property_num = 1:2:nargin-2
    switch varargin{property_num}
        case 'color'
            property_name = [property_name(:)',{'FaceColor'}];
            property_value = [property_value(:)',varargin{property_num+1}];
        case 'ArrowLineWidth'
            if isreal(varargin{property_num+1})
                arrow_line_width = varargin{property_num+1};
            else
                warning('vis_arrow:ArrowLineWidth','ArrowLineWidth must be a real number');
            end
        case 'ArrowHeadWidth'
            if isreal(varargin{property_num+1})
                arrow_head_width = varargin{property_num+1};
            else
                warning('vis_arrow:ArrowHeadWidth','ArrowHeadWidth must be a real number');
            end
        otherwise
            property_name = [property_name(:)',varargin{property_num}];
            property_value = [property_value(:)',varargin{property_num+1}];
    end
end

%%% Parameters setting
if ~exist('arrow_line_width','var') % Default value
    ax = axis;
    if numel(ax) == 4
        arrow_line_width = norm(ax([2 4])-ax([1 3]))/300;
    elseif numel(ax) == 6
        arrow_line_width = norm(ax([2 4 6])-ax([1 3 5]))/300;
    end
else    % Convert the unit from [m] to [mm]
    arrow_line_width = arrow_line_width/1000;
end
if ~exist('arrow_head_width','var') % Default value
    arrow_head_width = 3*arrow_line_width;
else    % Convert the unit from [m] to [mm]
    arrow_head_width = arrow_head_width/1000;
end
arrow_head_angle = 45.0/180*pi;
arrow_head_length = arrow_head_width/tan(arrow_head_angle/2);
% Number of faces that make up arrow (more -> refined)
arrow_line_face_num = 6;    % cylinder
arrow_head_face_num = 6;    % cone

%%% Basic lengths and vectors
% Unit vector in arrow direction
x = (end_point-start_point)/norm(end_point-start_point);
% y and z are unit vectors orthogonal to arrow
y = cross(x,[0;0;1]);
if norm(y) < 0.1
    y = cross(x,[0;1;0]);
end
y = y/norm(y);
z = cross(x,y);
z = z/norm(z);

%%% Basic angles
% List of angles from 0 to 2*pi for circle of cylinder
theta = 0:2*pi/arrow_line_face_num:2*pi;
sin_theta = sin(theta);
cos_theta = cos(theta);
% List of angles from 0 to 2*pi for circle of cone
phi = 0:2*pi/arrow_head_face_num:2*pi;
sin_phi = sin(phi);
cos_phi = cos(phi);

%%% Initialize arrow face matrix
arrow_component.faces = NaN([arrow_line_face_num+arrow_head_face_num+2 arrow_head_face_num+1]);
arrow_component.vertices = NaN([2*arrow_line_face_num+arrow_head_face_num+4 length(start_point)]);

%%% Normal arrow
if norm(end_point-start_point) > arrow_head_length
    % Vertices of the first cylinder circle
    for idx = 1:arrow_line_face_num+1
        arrow_component.vertices(idx,:) = start_point + arrow_line_width*(sin_theta(idx)*y + cos_theta(idx)*z);
    end
    % Vertices of the second cylinder circle
    arrow_line_end_point = end_point - arrow_head_length*x;
    for idx = 1:arrow_line_face_num+1
        arrow_component.vertices(arrow_line_face_num+1+idx,:) = arrow_line_end_point + arrow_line_width*(sin_theta(idx)*y + cos_theta(idx)*z);
    end
    % Vertices of the cone circle
    for idx = 1:arrow_head_face_num+1
        arrow_component.vertices(2*arrow_line_face_num+2+idx,:) = arrow_line_end_point + arrow_head_width*(sin_phi(idx)*y + cos_phi(idx)*z);
    end
    % Vertex of the cone tip
    arrow_component.vertices(2*arrow_line_face_num+arrow_head_face_num+4,:) = end_point;

    % Face of the cylinder circle
    arrow_component.faces(1,1:arrow_line_face_num+1) = 1:arrow_line_face_num+1;
    % Faces of the cylinder
    for idx = 1:arrow_line_face_num
        arrow_component.faces(1+idx,1:4) = [idx idx+1 arrow_line_face_num+1+idx+1 arrow_line_face_num+1+idx];
    end
    % Face of the cone circle
    arrow_component.faces(arrow_line_face_num+2,:) = 2*arrow_line_face_num+3:(2*arrow_line_face_num+3)+arrow_head_face_num;
    % Faces of the cone
    for idx = 1:arrow_head_face_num
        arrow_component.faces(arrow_line_face_num+2+idx,1:3) = [2*arrow_line_face_num+2+idx 2*arrow_line_face_num+2+idx+1 2*arrow_line_face_num+arrow_head_face_num+4];
    end
%%% Only cone
else
    arrow_head_width = 2*sin(arrow_head_angle/2)*norm(end_point-start_point);
    % Vertices of the cone circle
    for idx = 1:arrow_head_face_num+1
        arrow_component.vertices(idx,:) = start_point + arrow_head_width*(sin_phi(idx)*y + cos_phi(idx)*z);
    end
    % Vertex of the cone tip
    arrow_component.vertices(arrow_head_face_num+2,:) = end_point;
    % Face of the cone circle
    arrow_component.faces(1,:) = 1:arrow_head_face_num+1;
    % Faces of the cone
    for idx = 1:arrow_head_face_num
        arrow_component.faces(1+idx,1:3) = [idx idx+1 arrow_head_face_num+2];
    end
end

%%% Draw an arrow
arrow = patch(arrow_component);
for property_num = 1:numel(property_name)
    try
        set(arrow,property_name{property_num},property_value{property_num});
    catch
        disp(lasterr)
    end
end