%%%%%% Grid Map Designer
%%%%%% main_grid_map_designer.m
%%%%%% 
%%%%%% Main File
%%%%%%
%%%%%% Create grid maps with void and thinned areas.
%%%%%%
%%%%%% Created 2021-01-26
%%%%%% Keigo Haji
%%%%%% Last update: 2021-06-11
%%%%%% Kentaro Uno (made a new 3m x 3m map for the example demo 3)
% 
% Create grid maps with void and thinned areas.
% 
% [Note]
% This file is NOT used in main_sim.m.
% This file is used only before running climb_main to create a proper
% environment file as .mat file.
% You can design a grid-style environment with any stride width, any size,
% any thinning percentage, and any void area. 
% In the current version, both the grid map and the internal blank and
% thinned regions are created as squares.
%
% Variables
%   field: A structure for creating a field.
%   thin: A stucture for creating thinned areas.
%   void: A structure for creating void areas.
%   map: A structure for drawing points in the designed grid map.
%   x: 1*n vector 
%   y: 1*n vector
%   z: n*n matrix
%
% OUTPUT
%   Executing this file will save the three variables of x-y-z as a single
%   .mat file in this folder with the specified file name. 
%
close all;
clear;

%%%%%%%%%%%%%%%%%%%%%%%%%% Parameter Setting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Set the file name as you want to save.
filename = 'test_grid_map_3mx3m_dx85mm_thined_20_2void_areas.mat';  %'xxx.mat'

%Set the stride width
field.dx = 85;   %[mm]
%Set the field size
field.size = 3;  %[m]


%Set the switch for thinning
thin.setting = 'full';    %'on', 'off', or 'full', when "full" is selected, the entire map is targeted.
%Set the number of thinning areas
thin.num = 1;
%Set the center positions of thinning areas as a matrix. 
%Each row indicates each center and columns indicate x and y positions. 
% thin.center = [field.size/6 field.size/6;
%                field.size/2 field.size/6;
%                field.size/6*5 field.size/6;
%                field.size/6 field.size/2;
%                field.size/2 field.size/2;
%                field.size/6*5 field.size/2;
%                field.size/6 field.size/6*5;
%                field.size/2 field.size/6*5;
%                field.size/6*5 field.size/6*5
%                ]; %[m m]
thin.center = [3*field.size/4 field.size/4];
%Set the thinning percentage as a vector.
% This number is the percentage of points to be thinned out, not the
% percentage to be left behind; 0% does nothing, 100% is theoretically
% void.  
% thin.percentage = [50 50 50 50 90 50 50 50 50]; % [%] 0 - 100
thin.percentage = 20; % [%] 0 - 100
%Set the length of one side of thinning area
thin.length = 1.5;    %[m]

%Set the switch for void area
void.setting = 'on';    %'on' or 'off'
%Set the number of void areas
void.num = 2;
%Set the center positons of void squares as a matrix
%Each row indicates each center and columns indicate x and y positions. 
% void.center =[field.size/6 field.size/6;
%               field.size/2 field.size/6]; %[m m]
void.center =[ 0.95 0.9;
               2.05 2.15]; %[m m]
%Set the length of one side of void area
% void.length = 1.0;    %[m]
void.length_xy = [0.3 1.4;
                  0.3 1.4];      %[m]
              
%%%%%%%%%%%%%%%%%%%%%%% Create the grid map %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Convert units
field.dX = field.dx / 1000;  %[m] 

%Make Grid area
[field.X, field.Y] = meshgrid(-field.size/2 : field.dX : field.size/2);
%Convert formats
x = field.X(1,:);
y = field.Y(:,1)';
%Insert z positons
z = zeros(length(x));
z(:,:) = 0; %previously -0.08


%%%%%%%%%%%%%%%%% Thinned points %%%%%%%%%%%%%%%%%
if strcmp(thin.setting,'full') || strcmp(thin.setting,'on')
    if strcmp(thin.setting,'full')
        thin.id_xmax = length(x);
        thin.id_xmin = 1;
        thin.id_ymax = length(y);
        thin.id_ymin = 1;
        thin.num_of_thinning_points = thin.percentage/100 * numel(z(thin.id_xmin : thin.id_xmax, thin.id_ymin : thin.id_ymax));
        %Loop thinning until the specified percentage is reached
        while sum(sum(isnan(z(thin.id_xmin : thin.id_xmax, thin.id_ymin : thin.id_ymax)))) < thin.num_of_thinning_points
            thin.id_x = randi([thin.id_xmin, thin.id_xmax]);
            thin.id_y = randi([thin.id_ymin, thin.id_ymax]);
            z(thin.id_x, thin.id_y) = NaN;
        end
    end
    if strcmp(thin.setting,'on')
        %Convert units
        thin.id_length = round(thin.length/field.dX);
        for i = 1:thin.num
            %Convert units
            thin.id_center(i,:) = round(thin.center(i,:)/field.dX);
            
            %Set the void area in grid-format indeies
            thin.id_xmax(i) = round(thin.id_center(i,1) + thin.id_length/2 + 1);
            thin.id_xmin(i) = round(thin.id_center(i,1) - thin.id_length/2 + 1);
            thin.id_ymax(i) = round(thin.id_center(i,2) + thin.id_length/2 + 1);
            thin.id_ymin(i) = round(thin.id_center(i,2) - thin.id_length/2 + 1);
            thin.num_of_thinning_points = thin.percentage(i)/100 * numel(z(thin.id_xmin(i) : thin.id_xmax(i), thin.id_ymin(i) : thin.id_ymax(i)));
            %Loop thinning until the specified percentage is reached
            while sum(sum(isnan(z(thin.id_xmin(i) : thin.id_xmax(i), thin.id_ymin(i) : thin.id_ymax(i))))) < thin.num_of_thinning_points
                thin.id_x = randi([thin.id_xmin(i), thin.id_xmax(i)]);
                thin.id_y = randi([thin.id_ymin(i), thin.id_ymax(i)]);
                z(thin.id_x, thin.id_y) = NaN;
            end
        end
        
    end
end

%%%%%%%%%%%%%%%%%%%%%% Void Area  %%%%%%%%%%%%%%%%%%%%%%
if strcmp(void.setting, 'on')
%     void.id_length = void.length/field.dX;
    void.id_length_xy = void.length_xy/field.dX;
%     %%% Create void area %%%
%     for i = 1:void.num
%         %Convert units
%         void.id_center(i,:) = round(void.center(i,:)/field.dX);
%         
%         %Set the void area in grid-format indeies
%         void.id_xmax(i) = void.id_center(i,1) + void.id_length/2 +1;
%         void.id_xmin(i) = void.id_center(i,1) - void.id_length/2 +1;
%         void.id_ymax(i) = void.id_center(i,2) + void.id_length/2 +1;
%         void.id_ymin(i) = void.id_center(i,2) - void.id_length/2 +1;
%         
%         % Insert NaN in the void area
%         z(void.id_xmin(i):void.id_xmax(i),void.id_ymin(i):void.id_ymax(i)) = NaN;
%     end
    %%% Create void area %%%
    for i = 1:void.num
        %Convert units
        void.id_center(i,:) = round(void.center(i,:)/field.dX);
        
        %Set the void area in grid-format indeies
        void.id_xmax(i) = void.id_center(i,1) + void.id_length_xy(i,1)/2 +1;
        void.id_xmin(i) = void.id_center(i,1) - void.id_length_xy(i,1)/2 +1;
        void.id_ymax(i) = void.id_center(i,2) + void.id_length_xy(i,2)/2 +1;
        void.id_ymin(i) = void.id_center(i,2) - void.id_length_xy(i,2)/2 +1;
        
        % Insert NaN in the void area
        z(void.id_xmin(i):void.id_xmax(i),void.id_ymin(i):void.id_ymax(i)) = NaN;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Save the map %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
save(filename,'x','y','z')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% View the map %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[map.n,map.m] = size(z);
map_points = zeros(3,map.n*map.m);
map_points(1,:) = reshape(field.X,1,map.n*map.m);
map_points(2,:) = reshape(field.Y,1,map.n*map.m);
map_points(3,:) = reshape(z,1,map.n*map.m);
figure(1);
map_points = rmmissing(map_points,2);
plot3(map_points(1,:),map_points(2,:),map_points(3,:),'g.')


