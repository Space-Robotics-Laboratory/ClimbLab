%%%%%% Initialize
%%%%%% vis_create_robot_model
%%%%%% 
%%%%%% Create links of the robot for 3D plot
%%%%%% 
%%%%%% Created 2020-04-08
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-17
%
%
% Create the shapes for the links of the robot. Each link is made from a hexagonal shape and the base is a rectangular one.
%
% Function variables:
%
%     OUTPUT
%         shape_robot        : Vertices and faces indicators for base and legs (structure)
%         shape_robot.V_leg  : Vertices for legs' links (12x3xLP.num_q/LP.num_limb matrix)
%         shape_robot.F_leg  : Faces numbering for legs' links (10x4 matrix)
%         shape_robot.grip_F : Faces numbering for gripper parts (6x4 matrix)
%         shape_robot.grip_V : Vertices for gripper parts (8x3)
%         shape_robot.base_V : Vertices for base (8x3)
%         shape_robot.base_F : Faces numbering for base (6x4 matrix)
%     INPUT 
%         LP                 : Link parameters (SpaceDyn class)
%         F_grip             : Maximum gripping force [N] (scalar)
%         ani_settings.link_radius    : Radius for the links [m] (scalar)
%         ani_settings.base_thickness : Thickness of the base [m] (scalar)

function shape_robot = vis_create_robot_model(ani_settings, LP, F_grip)

if strcmp(ani_settings.display,'on')  
  
  % Create links
  for i = 1:3
      for j = 1:6
          shape_robot.V_leg(j,:,i) = (rpy2dc([(j-1)/3*pi;0;0])'*[LP.cc(1,i,i); ani_settings.link_radius;0])';
      end
      for j = 7:12
          shape_robot.V_leg(j,:,i) = (rpy2dc([(j-1)/3*pi;0;0])'*[-LP.cc(1,i,i); ani_settings.link_radius;0])';
      end
      shape_robot.F_leg(:,:,i) = [1 2 3 6;
                      3 4 5 6;
                      1 2 8 7;
                      2 3 9 8;
                      3 4 10 9;
                      4 5 11 10;
                      5 6 12 11;
                      6 1 7 12;
                      7 8 9 12;
                      9 10 11 12];
  end

  if F_grip > 0
      % Create gripper
      shape_robot.grip_F = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];
      finger_length = 1.75*ani_settings.link_radius;
      finger_thickness = 0.75*ani_settings.link_radius;
      shape_robot.grip_V = [-finger_length  finger_thickness/2  0.8*finger_thickness;
                             finger_length  finger_thickness/2  0.8*finger_thickness;
                             finger_length -finger_thickness/2  0.8*finger_thickness;
                            -finger_length -finger_thickness/2  0.8*finger_thickness;
                            -finger_length  finger_thickness/2 -0.2*finger_thickness;
                             finger_length  finger_thickness/2 -0.2*finger_thickness;
                             finger_length -finger_thickness/2 -0.2*finger_thickness;
                            -finger_length -finger_thickness/2 -0.2*finger_thickness];
  end

  % Create base
  for i = 1:LP.num_limb
      shape_robot.base_V(i,:) = [LP.c0(1:2,3*i-2)'  ani_settings.base_thickness/2];
  end
  for i = 1:LP.num_limb
      shape_robot.base_V(LP.num_limb+i,:) = [LP.c0(1:2,3*i-2)'  -ani_settings.base_thickness/2];
  end
  shape_robot.base_F = [1 2 3 4; 1 2 6 5; 2 3 7 6; 3 4 8 7; 1 4 8 5; 5 6 7 8];


  else
    shape_robot = [];
end

end