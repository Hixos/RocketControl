within RocketControl.Blocks.Math.Matrices;

model MatrixMult
  extends RocketControl.Icons.MatrixBlock;
  parameter Integer n(min = 1) = 1 annotation(
    Evaluate = true);
  parameter Integer m(min = 1) = n annotation(
    Evaluate = true);
  Modelica.Blocks.Interfaces.RealInput x[m] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, -80}, {-100, -40}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput M[n, m] annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, 40}, {-100, 80}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput Mx[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
equation
  Mx = M * x;
  annotation(
    Icon(graphics = {Text(origin = {-5, 2}, extent = {{-81, 74}, {81, -74}}, textString = "M*x"), Text(origin = {-73, 61}, lineColor = {102, 102, 102}, extent = {{-27, 21}, {27, -21}}, textString = "M"), Text(origin = {-73, -61}, lineColor = {102, 102, 102}, extent = {{-27, 21}, {27, -21}}, textString = "x"), Text(origin = {10, -10}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end MatrixMult;
