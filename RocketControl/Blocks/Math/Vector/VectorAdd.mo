within RocketControl.Blocks.Math.Vector;

model VectorAdd
  extends Internal.VectorIcon;
  parameter Integer n(min = 1) = 3 annotation(
    Evaluate = true);
  parameter Real gain[2] = {1, 1};
  Modelica.Blocks.Interfaces.RealOutput vc[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v2[n] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v1[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  vc = gain[1] * v1 + gain[2] * v2;
  annotation(
    Icon(graphics = {Line(points = {{-60, 0}, {60, 0}}, thickness = 2), Line(points = {{0, 60}, {0, -60}}, thickness = 2), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end VectorAdd;
