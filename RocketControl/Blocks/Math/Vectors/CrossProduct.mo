within RocketControl.Blocks.Math.Vectors;

model CrossProduct
  extends RocketControl.Icons.VectorBlock;
  Modelica.Blocks.Interfaces.RealOutput vc[3] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v2[3] annotation(
    Placement(visible = true, transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput v1[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  vc = cross(v1, v2);
  annotation(
    Icon(graphics = {Line(points = {{-60, 60}, {60, -60}, {60, -60}}, thickness = 2), Line(points = {{60, 60}, {-60, -60}}, thickness = 2), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name")}));
end CrossProduct;
