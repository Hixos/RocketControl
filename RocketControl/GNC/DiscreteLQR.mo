within RocketControl.GNC;

block DiscreteLQR
  extends Interfaces.PartialConditionalEnablePort;
  
  parameter SI.Duration dt;
  
  parameter Integer n(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer m(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer maxiter(min = 1) = 50;
  parameter Real tol(min = 0) = 1e-13;
  parameter Real s = 1;
    Modelica.Blocks.Interfaces.RealInput A[n, n](start = zeros(n,n)) annotation(
    Placement(visible = true, transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput B[n, m](start = zeros(n,m)) annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput Q[n, n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput R[m, m] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput u[m](start = zeros(m)) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput x[n](start = zeros(n)) annotation(
    Placement(visible = true, transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput K[m, n](start = zeros(m,m)) annotation(
    Placement(visible = true, transformation(origin = {110, 62}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

Real Ad[n,n];
Real Bd[n,m];
Real x_prev[n];

Real P[n,n];

Integer niter(start = 0);
equation
  when Clock() then
    Ad = identity(n) + sample(A)*dt + 0.5*sample(A)^2*dt^2;
    Bd = sample(B)*dt + 0.5*sample(A)*sample(B)*dt^2; 
    x_prev = previous(x);
    (P, niter) = RocketControl.Math.Matrices.sdaDare(Ad, Bd, R, Q, s, tol, maxiter);
    K = Modelica.Math.Matrices.solve2(R+transpose(Bd)*P*Bd, transpose(Bd)*P*Ad);
    u = -K*x_prev;
  end when;


  annotation(
    Icon(graphics = {Polygon(fillColor = {247, 219, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {-100, 100}, {-100, -80}, {-100, -100}, {70, -100}, {100, -80}, {100, 80}, {70, 100}, {-80, 100}}), Text(origin = {-80, -80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "x"), Text(origin = {80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "u"), Text(origin = {-80, 40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "B"), Rectangle(origin = {-100, -80}, fillColor = {217, 255, 215}, fillPattern = FillPattern.Solid, extent = {{-40, 20}, {40, -20}}), Text(origin = {-85.999, 138}, lineColor = {128, 128, 128}, extent = {{99.9988, -29}, {135.999, -58}}, textString = "e"), Text(origin = {-80, 0}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "Q"), Text(origin = {-80, -40}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "R"), Text(origin = {-2, 1}, extent = {{-60, 59}, {60, -59}}, textString = "LQ"), Text(origin = {-80, 80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "A"), Text(origin = {80, 52}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "K"), Text(origin = {-80, -80}, lineColor = {102, 102, 102}, extent = {{-20, 20}, {20, -20}}, textString = "x")}));
end DiscreteLQR;
