classdef SeventhOrderBezier < handle
% SeventhOrderBezier
% Calculate seventh order bezier trajectory coefficients
%
% Created     : 2021.08.21 by Warley Ribeiro
% Last updated: 2024.12.12 by Masazumi Imai

  %% Properties
  properties (Constant, GetAccess = private)
    % Order of the Bezier polynomial
    kOrder_ (1, 1) uint8 = 7;
  end
  properties (SetAccess = private, GetAccess = public)
    % Control points
    %   1st dim: x-y-z coordinates
    %   2nd dim: control points index
    control_points_ (3, 8) double;
  end

  %% Public Methods
  methods (Access = public)

    function seventh_order_bezier = SeventhOrderBezier()
    % SeventhOrderBezier() Constructor
      seventh_order_bezier.control_points_ = zeros(3, seventh_order_bezier.kOrder_ + 1);
    end

    function calcControlPoints(seventh_order_bezier, ...
        time_constraints, position_constraints, velocity_constraints, acceleration_constraints)
      arguments (Input)
        seventh_order_bezier;
        time_constraints         (1, :) {mustBeA(time_constraints, "double")};
        position_constraints     (3, :) {mustBeA(position_constraints, "double")};
        velocity_constraints     (3, :) {mustBeA(velocity_constraints, "double")};
        acceleration_constraints (3, :) {mustBeA(acceleration_constraints, "double")};
      end

      n = double(seventh_order_bezier.kOrder_);
      AA = zeros(3, n + 1);

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

      % Computing coefficients for position constraints
      AA(:, 1) = start_position;
      AA(:, n + 1) = final_position;
      % Computing coefficients for velocity constraints
      AA(:, 2) = start_velocity ./ n + AA(:, 1);
      AA(:, n) = -final_velocity ./ n + AA(:, n + 1);
      % Computing coefficients for acceleration constraints
      AA(:, 3)     = start_acceleration ./ (n * (n - 1)) - AA(:, 1) + 2 * AA(:, 2);
      AA(:, n - 1) = final_acceleration ./ (n * (n - 1)) - AA(:, n + 1) + 2 * AA(:, n);

      x = zeros(3, 1);
      for i = 0 : n
        x = x + nchoosek(n, i) * (mid_time / final_time).^i ...
          .* ((final_time - mid_time) / final_time).^(n - i) .* AA(1:3, i + 1);
      end
      xd = zeros(3, 1);
      for j = 0 : n - 1
        xd = xd + nchoosek(n - 1, j) * (mid_time / final_time).^j ...
          .* ((final_time - mid_time) / final_time).^(n - 1 - j) * n / final_time ...
          .* (AA(1:3, j + 2) - AA(1:3, j + 1));
      end

      XX = [mid_position' - x';
            mid_velocity' - xd'];

      % Coefficients
      t_d4 = n * (final_time - start_time) ...
        * ( seventh_order_bezier.calcBernsteinPolynomial( ...
                  start_time, final_time, mid_time, 2, n - 1) - ...
            seventh_order_bezier.calcBernsteinPolynomial( ...
                  start_time, final_time, mid_time, 3, n - 1) );
      t_d5 = n * (final_time - start_time) ...
        * ( seventh_order_bezier.calcBernsteinPolynomial( ...
                  start_time, final_time, mid_time, 3, n - 1) - ...
            seventh_order_bezier.calcBernsteinPolynomial( ...
                  start_time, final_time, mid_time, 4, n - 1) );

      TT = [seventh_order_bezier.calcBernsteinPolynomial( ...
                      start_time, final_time, mid_time, 3, n), ...
                  seventh_order_bezier.calcBernsteinPolynomial( ...
                            start_time, final_time, mid_time, 4, n);
            t_d4, t_d5];

      AA(1:3, 4:5) = (TT \ XX)';

      seventh_order_bezier.control_points_ = AA;
    end

    function desired_position = calcDesiredPositionForCurrentTimeStep(seventh_order_bezier, ...
        current_time, start_time, final_time)
      arguments (Input)
        seventh_order_bezier;
        current_time (1, 1) {mustBeA(current_time, "double")};
        start_time   (1, 1) {mustBeA(start_time,   "double")};
        final_time   (1, 1) {mustBeA(final_time,   "double")};
      end

      n = double(seventh_order_bezier.kOrder_);
      x = zeros(3, 1);
      % Compute position from the Bezier curve equation
      for j = 0 : n
        control_points = seventh_order_bezier.control_points_(:, j + 1);
        bernstein_polynomial = ...
          seventh_order_bezier.calcBernsteinPolynomial(start_time, final_time, current_time, j, n);
        x = x + control_points .* bernstein_polynomial;
      end
      desired_position = x;
    end

  end

  %% Private Methods
  methods (Access = private)

    function Bj = calcBernsteinPolynomial(~, t0, tf, t, j, n)
      Bj = nchoosek(n, j) * ((t - t0) / (tf - t0)).^j .* ((tf - t) / (tf - t0)).^(n - j);
    end

  end

end  % SeventhOrderBezier
