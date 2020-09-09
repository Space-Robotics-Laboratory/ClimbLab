%%%%%% Update
%%%%%% upd_ground_reaction_forces_spring_damper
%%%%%% 
%%%%%% Obtain external force on each limb using spring damper model
%%%%%% 
%%%%%% Created 2020-04-09
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-09
%
%
% Obtain external forces on each limb based on a spring-damper model for the contact, which is based on the first contact
% point between the robot and the surface.
%
%                                   F_e = -K (x_e - x_c) - D dx_e/dt
%
%         F_e : Ground reaction force
%         K   : Ground stiffness coefficient
%         D   : Ground damping coefficient
%         x_e : Current foot position
%         x_c : Contact position
%
%
% Function variables:
%
%     OUTPUT
%         SV.Fe            : External force on the link end-point [N] (3xn vector)
%     INPUT
%         LP               : Link Parameters (SpaceDyn class)
%         SV               : State Variables (SpaceDyn class)
%         surface_param.K  : Floor reaction force stiffness coefficient (scalar)
%         surface_param.D  : Floor reaction force damping coefficient (scalar)
%         POS_e            : Position of the end-effector [m] (3xnum_limb matrix)
%         cont_POS         : Initial point of contact which is used as equilibrium position [m] (3xnum_limb matrix)

function SV = upd_ground_reaction_forces_spring_damper(LP, SV, surface_param, POS_e, cont_POS)

global contact_f

% Calculate floor reaction force in each limb end-effector
for i = 1:LP.num_limb
    if contact_f(i)
        % Calculate end-effector velocity
        J(:,:,i) = calc_gj(LP,SV,i);
        VEL_e(:,i) = J(:,:,i)*SV.qd;
        % Floor reaction force based on spring-damper model
        SV.Fe(:,3*i) = -surface_param.K*(POS_e(:,i) - cont_POS(:,i)) - surface_param.D*VEL_e(1:3,i);
    else
        SV.Fe(:,3*i) = [0;0;0];
    end
end
    
end