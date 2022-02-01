within RocketControl.Blocks.Math.Vector;

model VectorNorm
  extends Internal.VectorIcon;
  parameter Integer n(min = 1) = 3 annotation(
    Evaluate = true);
  Modelica.Blocks.Interfaces.RealInput v[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput vnorm annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
equation
  vnorm = norm(v);
  annotation(
    Icon(graphics = {Text(origin = {-1, 9}, extent = {{-73, 81}, {73, -81}}, textString = "|v|"), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end VectorNorm;
