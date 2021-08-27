%%%%%% Update
%%%%%% upd_base_next_pos_CoM_projection_on_intersection_of_diagonals
%%%%%%
%%%%%% Update the kinematic feasibility flag (true or false)
%%%%%%
%%%%%% Created: 2020-11-18
%%%%%% Kentaro Uno
%%%%%% Last update: 2021-04-11
%%%%%% Kentaro Uno
%
%
% You can see the details of the planning method is shown in the following paper:
% --------------------------------------------------------------------
% In: Proceedings of the IEEE/SICE International Symposium on System 
% Integration (SII) 2019 by K. Uno et al.
% Proceedings Paper URL: 
% https://ieeexplore.ieee.org/document/8700455
% --------------------------------------------------------------------
%
% Function variables:
%
%     OUTPUT
%         gait_planning_param   : Parameters for gait planning (class)
%
%         gait_planning_param.base_cur      : Current position of the base when selecting a new one [m] (3x1 vector)
%         gait_planning_param.base_next     : Next desired position of the base [m] (3x1 vector)
%         gait_planning_param.base_T        : Initial and final time for the movement between current and desired position [s] (2x1 vector)
%     INPUT
%         gait_planning_param   : Parameters for path planning (class)
%         surface_param         : Parameters for surface (class)
%         SV                    : State values (SpaceDyn class)
%         des_SV                : Desired state values (SpaceDyn class)
%         LP                    : Link parameters (SpaceDyn class)
%         POS_e                 : Position of the end-effector [m] (3xnum_limb matrix)
%         inc                   : Surface inclination [deg] (scalar) 
%         base_height           : Height of base relative to map [m]
%         time                  : Simulation time [s] (scalar)

function gait_planning_param = upd_base_next_pos_CoM_projection_on_intersection_of_diagonals(SV,des_SV,LP,gait_planning_param,POS_e,inc,surface_param,base_height,time)
    % If all legs are in support phase
    if sum(des_SV.sup) == LP.num_limb

    % Initialize the next position of the base to be planned
    gait_planning_param.base_next = SV.R0;

    % Update the current position of the base
    gait_planning_param.base_cur = SV.R0;

    % prepare some temporary variables in advance
    L1_x = zeros(2,1); % x positions of first diagonal line (connecting limb 1 and limb 3 supporting points)
    L1_y = zeros(2,1); % y positions of first diagonal line (connecting limb 1 and limb 3 supporting points)
    L2_x = zeros(2,1); % x positions of second diagonal line (connecting limb 2 and limb 4 supporting points)
    L2_y = zeros(2,1); % y positions of second diagonal line (connecting limb 2 and limb 4 supporting points)

    for j = 1:LP.num_limb
        if j == gait_planning_param.swing_number
            switch j
                case 1
                    L1_x(1) = gait_planning_param.POS_next(1,j);
                    L1_y(1) = gait_planning_param.POS_next(2,j);
                case 2
                    L2_x(1) = gait_planning_param.POS_next(1,j);
                    L2_y(1) = gait_planning_param.POS_next(2,j);
                case 3
                    L1_x(2) = gait_planning_param.POS_next(1,j);
                    L1_y(2) = gait_planning_param.POS_next(2,j);
                case 4
                    L2_x(2) = gait_planning_param.POS_next(1,j);
                    L2_y(2) = gait_planning_param.POS_next(2,j);
            end
        else
            switch j
                case 1
                    L1_x(1) = POS_e(1,j);
                    L1_y(1) = POS_e(2,j);
                case 2
                    L2_x(1) = POS_e(1,j);
                    L2_y(1) = POS_e(2,j);
                case 3
                    L1_x(2) = POS_e(1,j);
                    L1_y(2) = POS_e(2,j);
                case 4
                    L2_x(2) = POS_e(1,j);
                    L2_y(2) = POS_e(2,j);
            end
        end
    end

    % Calculate distances
    Dx12 = L1_x(1)-L1_x(2);
    Dx34 = L2_x(1)-L2_x(2);
    Dy12 = L1_y(1)-L1_y(2);
    Dy34 = L2_y(1)-L2_y(2);
    Dx24 = L1_x(2)-L2_x(2);
    Dy24 = L1_y(2)-L2_y(2);
    % Solve the equation
    ts = [Dx12 -Dx34; Dy12 -Dy34] \ [-Dx24; -Dy24];
    % Take weighted combinations of points on the line
    P = ts(1)*[L1_x(1); L1_y(1)] + (1-ts(1))*[L1_x(2); L1_y(2)];

    % figure();hold on;grid on;
    % plot(L1_x,L1_y);
    % plot(L2_x,L2_y);
    % plot(P(1),P(2),'ko');
    % plot(POS_e(1,:),POS_e(2,:),'bo');
    gait_planning_param.base_next(1,1) = P(1) + norm(base_height)*tan(inc*pi/180);
    gait_planning_param.base_next(2,1) = P(2);
    gait_planning_param.base_next(3,1) = base_height + surface_param.floor_level;

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Time for current and desired positions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Timing
    gait_planning_param.base_T = [time; time + gait_planning_param.T_d];

end