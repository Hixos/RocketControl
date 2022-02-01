within RocketControl.GNC;

block DiscreteKalmanFilter
  parameter Integer n(min = 1) "Number of states" annotation(
    Evaluate = true);
  parameter Integer m(min = 0) "Number of inputs" annotation(
    Evaluate = true);
  parameter Integer p(min = 1) "Number of outputs" annotation(
    Evaluate = true);
  parameter Real A[n, n];
  parameter Real B[n, max(m, 1)] = zeros(n, max(m, 1)) annotation(
    Dialog(enable = m > 0));
  parameter Real C[p, n];
  parameter Real P0[n, n] = identity(n) "Initial state covariance matrix";
  parameter Real Q[n, n] "Process covariance matrix";
  parameter Real R[p, p] "Output covariance matrix";
  parameter Real x0[n] "Initial state";
  Real P[n, n](start = P0) "P(k|k)";
  Real Pest[n, n](start = P0) "P(k|k-1)";
  Real L[n, p];
  Real x_pred[n](start = x0);
  Modelica.Blocks.Interfaces.RealInput y_meas[p] annotation(
    Placement(visible = true, transformation(origin = {-106, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput x_est[n](start = x0, each fixed = true) annotation(
    Placement(visible = true, transformation(origin = {106, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y_est[p](start = C * x0, each fixed = true) annotation(
    Placement(visible = true, transformation(origin = {106, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u[max(m, 1)] annotation(
    Placement(visible = true, transformation(origin = {-106, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.BooleanInput enable annotation(
    Placement(visible = true, transformation(origin = {0, 108}, extent = {{-20, -20}, {20, 20}}, rotation = -90), iconTransformation(origin = {0, 102}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  Modelica.Blocks.Interfaces.BooleanInput correct annotation(
    Placement(visible = true, transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -100}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
equation
  when Clock() then
    if m == 0 then
      u[1] = sample(0);
    end if;
    if not sample(enable) then
      Pest = P0;
      L = zeros(n, p);
      P = P0;
      x_pred = x0;
      x_est = x0;
      y_est = C * x_est;
    else
      Pest = A * previous(P) * transpose(A) + Q;
      x_pred = A * previous(x_est) + B * u;
      if correct then
        L = transpose(Modelica.Math.Matrices.solve2(transpose(C * Pest * transpose(C) + R), transpose(Pest * transpose(C))));
        P = Pest - L * C * Pest;
        x_est = x_pred + L * (y_meas - C * x_pred);
      else
        L = zeros(n, p);
        P = Pest;
        x_est = x_pred;
      end if;
      y_est = C * x_est;
    end if;
  end when;
  annotation(
    Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "KF"), Text(origin = {102, 58.24}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "y_meas"), Text(origin = {-22, -53}, lineColor = {128, 128, 128}, extent = {{-108, 43}, {-72, 18}}, textString = "u"), Text(origin = {300, 52.24}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "x_est"), Text(origin = {304, -69.76}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "y_est"), Text(origin = {246, -135.76}, lineColor = {128, 128, 128}, extent = {{-246, 56.76}, {-164, 23.76}}, textString = "corr")}));
end DiscreteKalmanFilter;
