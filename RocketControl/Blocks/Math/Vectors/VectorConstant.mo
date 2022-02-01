within RocketControl.Blocks.Math.Vectors;

block VectorConstant
  extends RocketControl.Icons.VectorBlock;
  parameter Integer n(min = 1) = 3 annotation(
    Evaluate = true);
  parameter Real[n] k;
  Modelica.Blocks.Interfaces.RealOutput v[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  v = k;
  annotation(
    Icon(graphics = {Text(origin = {-6, 10}, extent = {{-86, 78}, {86, -78}}, textString = "{k}"), Text(origin = {10, -10}, lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {10, -250}, extent = {{-150, 150}, {150, 110}}, textString = "%k")}));
end VectorConstant;
