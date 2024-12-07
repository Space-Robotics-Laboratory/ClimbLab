classdef FifthOrderBezier
% FifthOrderBezier
% Calculate fifth order bezier trajectory coefficients
%
% Created     : 2022.01.19 by Warley Ribeiro
% Last updated: 2024.12.07 by Masazumi Imai

  %% Properties
  properties (Constant, GetAccess = private)
    % Order of the Bezier polynomial
    kBezierPolynomialOrder_ (1, 1) uint8 = 5;
  end
  properties (SetAccess = private, GetAccess = public)
    % Coefficients (control points)
    %   1st dim: x-y-z coordinates
    %   2nd dim: coefficient index
    coefficients_ (3, 6) double;
  end

  %% Public methods
  methods (Access = public)

    function fifth_order_bezier = FifthOrderBezier()
    % FifthOrderBezier() Constructor
      fifth_order_bezier.coefficients_ = zeros(3, fifth_order_bezier.kBezierPolynomialOrder_ + 1);
    end

    function fifth_order_bezier = calcCoefficients(fifth_order_bezier, ...
        time_constraints, position_constraints, velocity_constraints, acceleration_constraints)
      arguments (Input)
        fifth_order_bezier;
        time_constraints;
        position_constraints     (3, :) {mustBeA(position_constraints, "double")};
        velocity_constraints     (3, :) {mustBeA(velocity_constraints, "double")};
        acceleration_constraints (3, :) {mustBeA(acceleration_constraints, "double")};
      end

      n = double(fifth_order_bezier.kBezierPolynomialOrder_);
      AA = zeros(3, n + 1);
      start_position = position_constraints(:, 1);
      final_position = position_constraints(:, 2);
      start_velocity = velocity_constraints(:, 1);
      final_velocity = velocity_constraints(:, 2);
      start_acceleration = acceleration_constraints(:, 1);
      final_acceleration = acceleration_constraints(:, 2);

      % Computing coefficients for position constraints
      AA(:, 1) = start_position;
      AA(:, n + 1) = final_position;
      % Computing coefficients for velocity constraints
      AA(:, 2) = start_velocity ./ n + AA(:, 1);
      AA(:, n) = -final_velocity ./ n + AA(:, n + 1);
      % Computing coefficients for acceleration constraints
      AA(:, 3)     = start_acceleration ./ (n * (n - 1)) - AA(:, 1) + 2 * AA(:, 2);
      AA(:, n - 1) = final_acceleration ./ (n * (n - 1)) - AA(:, n + 1) + 2 * AA(:, n);

      % Unpacking coefficients to trajectory_planning struct
      for k = 1 : n + 1
        fifth_order_bezier.coefficients_(1:3, k) = AA(:, k);
      end
    end

    function desired_position = calcDesiredPositionForCurrentTimeStep(fifth_order_bezier, ...
        current_time, start_time, final_time)
      arguments (Input)
        fifth_order_bezier;
        current_time (1, 1) {mustBeA(current_time, "double")};
        start_time   (1, 1) {mustBeA(start_time,   "double")};
        final_time   (1, 1) {mustBeA(final_time,   "double")};
      end

      n = double(fifth_order_bezier.kBezierPolynomialOrder_);
      x = zeros(3, 1);
      % Compute position from the Bezier curve equation
      for j = 0 : n
        control_points = fifth_order_bezier.coefficients_(:, j + 1);
        bernstein_polynomial = ...
          fifth_order_bezier.calcBernsteinPolynomial(start_time, final_time, current_time, j, n);
        x = x + control_points .* bernstein_polynomial;
      end
      desired_position = x;
    end

  end

  %% Private methods
  methods (Access = private)

    function Bj = calcBernsteinPolynomial(~, t0, tf, t, j, n)
      Bj = nchoosek(n, j) * ((t - t0) / (tf - t0)).^j .* ((tf - t) / (tf - t0)).^(n - j);
    end

  end

end  % FifthOrderBezier
