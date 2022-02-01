within RocketControl.Blocks.Math.Matrix;

model MatrixConstant
  extends Internal.MatrixIcon;
  parameter Integer n(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer m(min = 1) = n annotation(
    Evaluate = true);
  parameter Real[n, m] val;
  Modelica.Blocks.Interfaces.RealOutput k[n, m] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  k = val;
  annotation(
    Icon(graphics = {Text(origin = {0, 10}, extent = {{-86, 78}, {86, -78}}, textString = "[k]"), Text(origin = {10, -10}, textColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end MatrixConstant;
