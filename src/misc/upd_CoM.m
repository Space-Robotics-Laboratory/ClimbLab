%%%%%% Update
%%%%%% upd_CoM
%%%%%% 
%%%%%% Obtain Center of Mass position
%%%%%% 
%%%%%% Created 2019-10-31
%%%%%% Warley Ribeiro
%%%%%% Last update: 2020-04-23
%
%
% Obtain center of mass position from links positions and inertia parameters
%
% Function variables:
%
%     OUTPUT
%         CoM          : Center of Mass position [m] (3x1 vector)
%     INPUT
%         LP           : Link Parameters (SpaceDyn class)
%         SV           : State Variables (SpaceDyn class)

function CoM = upd_CoM(LP, SV)

% Center of Mass calculation
CoM = LP.m0*SV.R0;
for i=1:LP.num_q
    CoM = CoM + LP.m(i)*SV.RR(:,i);
end
CoM = CoM/LP.mass;