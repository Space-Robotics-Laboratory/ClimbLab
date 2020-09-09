%%%%%% Update
%%%%%% upd_collision_endeffector
%%%%%% 
%%%%%% Obtain the contact point
%%%%%% 
%%%%%% Created 2020-04-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-09
%
%
% Detect a new contact (or losing an old one) between the robot and the ground surface. Obtain the contact position for
% further reactions calculation
%
% Function variables:
%
%     OUTPUT
%         cont_POS     : Initial point of contact which is used as equilibrium position [m] (3xnum_limb matrix)
%     INPUT
%         LP           : Link Parameters (SpaceDyn class)
%         SV           : State Variables (SpaceDyn class)
%         POS_e        : Position of the end-effector [m] (3xnum_limb matrix)
%         cont_POS     : Initial point of contact which is used as equilibrium position [m] (3xnum_limb matrix)

function cont_POS = upd_collision_endeffector(LP, SV, POS_e, cont_POS)


global contact_f

% Calculate floor reaction force in each limb end-effector
for i = 1:LP.num_limb
    % get contact point position 
    [cont_x,cont_y,cont_z] = get_map_pos(POS_e(1,i),POS_e(2,i));
    % Detect new contact or new grasp
    if contact_f(i) == 0 && (POS_e(3,i) <= cont_z || SV.sup(i) == 1)  
        contact_f(i) = 1;
        cont_POS(:,i) = [cont_x;cont_y; cont_z];
    end
    
    % Detect loosing contact condition
    if contact_f(i) == 1 && (POS_e(3,i) > cont_z && SV.sup(i) == 0)
        contact_f(i) = 0;
    end
    
end
    
end