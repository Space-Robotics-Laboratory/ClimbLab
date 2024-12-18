classdef SeventhOrderSpline < handle
% SeventhOrderSpline
% Calculate seventh order spline trajectory coefficients
%
% Created     : 2021.06.16 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (Constant, GetAccess = private)
    % Order of the Spline curve
    kOrder_ (1, 1) uint8 = 7;
  end
  properties (SetAccess = private, GetAccess = public)
    % Coefficients (coefficients)
    %   1st dim: x-y-z coordinates
    %   2nd dim: control points index
    control_points_ (3, 8) double;
  end

  %% Public Methods
  methods (Access = public)

    function seventh_order_spline = SeventhOrderSpline()
    % SeventhOrderSpline() Constructor
      seventh_order_spline.control_points_ = zeros(3, seventh_order_spline.kOrder_ + 1);
    end

    function calcControlPoints(seventh_order_spline, ...
      time_constraints, position_constraints, velocity_constraints, acceleration_constraints)
      arguments (Input)
        seventh_order_spline;
        time_constraints         (1, :) {mustBeA(time_constraints, "double")};
        position_constraints     (3, :) {mustBeA(position_constraints, "double")};
        velocity_constraints     (3, :) {mustBeA(velocity_constraints, "double")};
        acceleration_constraints (3, :) {mustBeA(acceleration_constraints, "double")};
      end

      start_time = time_constraints(1, 1);
      mid_time   = time_constraints(1, 2);
      final_time = time_constraints(1, end);
      start_position = position_constraints(:, 1);
      mid_position   = position_constraints(:, 2);
      final_position = position_constraints(:, end);
      start_velocity = velocity_constraints(:, 1);
      mid_velocity   = velocity_constraints(:, 2);
      final_velocity = velocity_constraints(:, end);
      start_acceleration = acceleration_constraints(:, 1);
      final_acceleration = acceleration_constraints(:, end);

      XX = [start_position'    ;
            mid_position'      ;
            final_position'    ;
            start_velocity'    ;
            mid_velocity'      ;
            final_velocity'    ;
            start_acceleration';
            final_acceleration'];

      ts = start_time;
      tm = mid_time;
      tf = final_time;
      TT = [1,  ts,    ts^2,    ts^3,     ts^4,     ts^5,     ts^6,     ts^7;
            1,  tm,    tm^2,    tm^3,     tm^4,     tm^5,     tm^6,     tm^7;
            1,  tf,    tf^2,    tf^3,     tf^4,     tf^5,     tf^6,     tf^7;
            0,   1,  2*ts  ,  3*ts^2,   4*ts^3,   5*ts^4,   6*ts^5,   7*ts^6;
            0,   1,  2*tm  ,  3*tm^2,   4*tm^3,   5*tm^4,   6*tm^5,   7*tm^6;
            0,   1,  2*tf  ,  3*tf^2,   4*tf^3,   5*tf^4,   6*tf^5,   7*tf^6;
            0,   0,  2     ,  6*ts  ,  12*ts^2,  20*ts^3,  30*ts^4,  42*ts^5;
            0,   0,  2     ,  6*tf  ,  12*tf^2,  20*tf^3,  30*tf^4,  42*tf^5];

      seventh_order_spline.control_points_ = (TT \ XX)';
    end

    function desired_position = calcDesiredPositionForCurrentTimeStep(seventh_order_spline, ...
        current_time, start_time, ~)
      arguments (Input)
        seventh_order_spline;
        current_time (1, 1) {mustBeA(current_time, "double")};
        start_time   (1, 1) {mustBeA(start_time,   "double")};
        ~;
      end
      desired_position = ...
          seventh_order_spline.control_points_(:, 1) ...
        + seventh_order_spline.control_points_(:, 2) * (current_time - start_time) ...
        + seventh_order_spline.control_points_(:, 3) * (current_time - start_time) .^ 2 ...
        + seventh_order_spline.control_points_(:, 4) * (current_time - start_time) .^ 3 ...
        + seventh_order_spline.control_points_(:, 5) * (current_time - start_time) .^ 4 ...
        + seventh_order_spline.control_points_(:, 6) * (current_time - start_time) .^ 5 ...
        + seventh_order_spline.control_points_(:, 7) * (current_time - start_time) .^ 6 ...
        + seventh_order_spline.control_points_(:, 8) * (current_time - start_time) .^ 7;
    end

  end

end  % SeventhOrderSpline
