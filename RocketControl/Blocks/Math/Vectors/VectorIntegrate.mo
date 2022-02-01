within RocketControl.Blocks.Math.Vectors;

block VectorIntegrate
  extends RocketControl.Icons.VectorBlock;
  parameter Integer n(min = 1) = 3 annotation(
    Evaluate = true);
  parameter Real k = 1;
  Modelica.Blocks.Interfaces.RealInput v[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput vi[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  der(vi) = k * v;
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {23, 10}, extent = {{-51, 64}, {51, -64}}, textString = "kv"), Line(origin = {-42.4, 11.56}, points = {{12.006, 67.944}, {-1.994, 69.944}, {-13.994, 51.944}, {14.006, -60.056}, {-1.994, -76.056}, {-17.994, -72.056}}, thickness = 1.25, smooth = Smooth.Bezier)}));
end VectorIntegrate;
