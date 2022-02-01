within RocketControl.Blocks.Math.Vectors;

model VectorSplit
 extends RocketControl.Icons.VectorBlock;
  parameter Integer n(min = 1) = 3 annotation(
    Evaluate = true);
parameter Integer s(min = 1, max = n-1) = 1 annotation(
    Evaluate = true);
  Modelica.Blocks.Interfaces.RealOutput va[s] annotation(
    Placement(visible = true, transformation(origin = {110, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{100, 30}, {120, 50}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
 Modelica.Blocks.Interfaces.RealOutput vb[n-s] annotation(
    Placement(visible = true, transformation(origin = {110, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(extent = {{100, -50}, {120, -30}}, rotation = 0)));
equation
  va = v[1:s];
  vb = v[s+1:end];
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Line(origin = {4, 21}, points = {{-104, -21}, {-4, -21}, {-4, 19}, {96, 19}, {104, 21}}), Line(origin = {50, -20}, points = {{50, -20}, {-50, -20}, {-50, 20}})}));
end VectorSplit;
