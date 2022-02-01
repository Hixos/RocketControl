within RocketControl.GNC;

block ContinuousLQR
  extends RocketControl.Interfaces.PartialConditionalEnablePort;
  parameter Integer n(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer m(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer maxiter(min = 1) = 50;
  parameter Real tol(min = 0) = 1e-13;
  parameter Real g = -1;
  Modelica.Blocks.Interfaces.RealInput A[n, n] annotation(
    Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput B[n, m] annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Q[n, n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput R[m, m] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput u[m](start = zeros(m)) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput x[n] annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Real P[n, n];
  Integer niter;
  Modelica.Blocks.Interfaces.RealOutput K[m, n] annotation(
    Placement(visible = true, transformation(origin = {110, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  if enable then
    (P, niter) = RocketControl.Math.sdaCare(A, B, R, Q, g, tol, maxiter);
    K = Modelica.Math.Matrices.solve2(R, transpose(B) * P);
    u = -K * x;
//  assert(niter < maxiter, "sdaCare maximum number of iterations reached", level = AssertionLevel.warning);
//      when niter == maxiter then
//       terminate("Too many iterations");
//      end when;
  else
    P = zeros(n, n);
    K = zeros(m, n);
    niter = 0;
    u = zeros(m);
  end if;
  annotation(
    Icon(graphics = {Polygon(fillColor = {247, 219, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-100, 100}, {-100, -80}, {-100, -100}, {70, -100}, {100, -80}, {100, 80}, {70, 100}, {-80, 100}}), Text(origin = {-2, 1}, extent = {{-60, 59}, {60, -59}}, textString = "LQ"), Text(origin = {-80, 80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "A"), Text(origin = {-80, 40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "B"), Text(origin = {-80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "Q"), Text(origin = {-80, -40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "R"), Text(origin = {80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "u"), Rectangle(origin = {-100, -80}, fillColor = {217, 255, 215}, fillPattern = FillPattern.Solid, extent = {{-40, 20}, {40, -20}}), Text(origin = {-80, -80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "x"), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {80, 52}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "K")}));
end ContinuousLQR;
