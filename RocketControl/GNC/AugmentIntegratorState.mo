within RocketControl.GNC;

block AugmentIntegratorState
  parameter Integer n(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer m(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer p(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Boolean use_D = false;
  Modelica.Blocks.Interfaces.RealInput B[n, m] annotation(
    Placement(visible = true, transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput A[n, n] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput C[p, n] annotation(
    Placement(visible = true, transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -30}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Aaug[n + p, n + p] annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Baug[n + p, m] annotation(
    Placement(visible = true, transformation(origin = {110, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Caug[p, n + p] annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput D[p, m] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  if not use_D then
    D = zeros(p, m);
  end if;
  Aaug = [A, zeros(n, p); -C, zeros(p, p)];
  Baug = [B; -D];
  Caug = [C, zeros(p, p)];
  annotation(
    Icon(graphics = {Text(origin = {-2, -250}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Rectangle(lineColor = {255, 85, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}, radius = 20), Text(origin = {-79, 81}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "A"), Text(origin = {7, -2}, extent = {{-73, 62}, {73, -62}}, textString = "der(e)"), Text(origin = {-80, -28}, lineColor = {102, 102, 102}, extent = {{-20, 18}, {20, -18}}, textString = "C"), Text(origin = {-79, 31}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "B"), Text(origin = {77, 49}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "Aaug"), Text(origin = {77, -45}, lineColor = {102, 102, 102}, extent = {{-21, 19}, {21, -19}}, textString = "Baug"), Text(origin = {-80, -78}, lineColor = {102, 102, 102}, extent = {{-20, 18}, {20, -18}}, textString = "D")}));
end AugmentIntegratorState;
