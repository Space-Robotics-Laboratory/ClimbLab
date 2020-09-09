%%%%%% Visualization of leg motion range
%%%%%%
%%%%%% function name:
%%%%%% drawRobotWireFrame.m
%%%%%% 
%%%%%% objective:
%%%%%% to visualize a base plate and 4 legs in wire frames to visualize a 
%%%%%% robot simply for better understanding of the reachable region 
%%%%%% 
%%%%%% input: 
%%%%%% - LP: link parameters class of the robot 
%%%%%% - SV: states valuables 
%%%%%% - floor:  
%%%%%% - POS_e: posion of endlinks (grippers)
%%%%%% - POS_j: position of each joints
%%%%%% - inc: 
%%%%%% - fig_num: figure number
%%%%%%
%%%%%% output:
%%%%%% - simple robot vizualization in figure(fig_num)
%%%%%%
%%%%%% Created/Updated Date:
%%%%%% 2019-10-02
%%%%%% 
%%%%%% Auther: 
%%%%%% Kentaro Uno, Naomasa Takada

function drawRobotWireFrame( LP , SV, floor, POS_e, POS_j, inc, fig_num ) 

%%% Figure initialization %%%
figure(fig_num);
clf;
hold on;
% Robot color
robotColor = [ 0.2, 0.2, 0.2 ];
% Surface inclination
surf_AA = rpy2dc([0;pi*inc/180;0])';

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ground surface drawing %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % Flat Surface initial point
% i = [-0.4;-0.4;floor];
% % Surface size
% s_x = [0.8;0;0];
% s_y = [0;0.8;0];
% s_z = [0;0;0.005];
% % Create surface vertices and faces
% surf_V = [i i+s_x i+s_x+s_y i+s_y i-s_z i-s_z+s_x i-s_z+s_x+s_y ...
%              i-s_z+s_y];
% surf_V = surf_AA'*surf_V;
% surf_F = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];
% % Draw surface
% patch('Vertices',surf_V','Faces',surf_F,'FaceColor',[0.6 0.6 0.6]);

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Robot base plate drawing %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Draw base plate
for i = 1:LP.num_limb
    base_V(i,:) = [LP.c0(1:2,3*i-2)'  0.01];
end
for i = 1:LP.num_limb
    base_V(LP.num_limb+i,:) = [LP.c0(1:2,3*i-2)'  -0.01];
end
base_F = [1 2 3 4;
          1 4 8 5;
          4 3 7 8;
          3 2 6 7;
          2 1 4 6;
          5 6 7 8];

% Robot base vertices change from actual robot position and orientation
base_V = base_V*SV.A0' + ones(size(base_V,1),1)*SV.R0';
base_V = base_V*surf_AA;

% Draw base
patch('Vertices',base_V,'Faces',base_F,'FaceColor',robotColor,'EdgeColor','w');
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Robot link frame drawing %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : LP.num_limb
    
    wireFrameX = zeros(1,4);
    wireFrameY = zeros(1,4);
    wireFrameZ = zeros(1,4);
    
    wireFrameX = POS_j( 1, 3*i-2:3*i );
    wireFrameY = POS_j( 2, 3*i-2:3*i );
    wireFrameZ = POS_j( 3, 3*i-2:3*i );
    
    wireFrameX = horzcat( wireFrameX, POS_e( 1, i ) );
    wireFrameY = horzcat( wireFrameY, POS_e( 2, i ) );
    wireFrameZ = horzcat( wireFrameZ, POS_e( 3, i ) );
    
    %%% draw links as a wire frame connecting between each joint
    plot3( wireFrameX, wireFrameY, wireFrameZ, '-o', 'LineWidth', 3, ...
       'MarkerEdgeColor','w', 'MarkerFaceColor',robotColor, 'MarkerSize',15, ...      
       'color', robotColor );
end
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Matlab figure setting    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lightangle(-10,15)
lighting gouraud;
material shiny; 
% Labels and axis dimensions
xlabel('x [m]','FontSize',16); ylabel('y [m]','FontSize',16);
zlabel('z [m]','FontSize',16); set(gca,'FontSize',16)
axis equal

grid on;
az = -10.00;
el = 15.00;
view(az,el);
end

%%% EOF %%%