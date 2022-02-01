within RocketControl.Blocks.Math.Vector;

block ProjectOnPlane
  extends Internal.VectorIcon;
  parameter Real n[3];
  Modelica.Blocks.Interfaces.RealInput v[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput vp[3] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  vp = v - v * n * n;
  annotation(
    Icon(graphics = {Polygon(origin = {-2, -30}, fillColor = {156, 227, 249}, fillPattern = FillPattern.Solid, points = {{-90, -38}, {-42, 38}, {90, 38}, {44, -38}, {30, -38}, {-90, -38}}), Line(origin = {-9.72603, 10.5616}, points = {{-26, -51}, {38, 73}}, thickness = 0.75, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {28.3562, 44.7123}, points = {{0, 39}, {0, -71}}, pattern = LinePattern.Dash, thickness = 0.75), Line(origin = {-9.43836, -28.1507}, points = {{-26, -12}, {38, 2}}, color = {255, 0, 0}, thickness = 0.75, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(lineColor = {0, 0, 255}, extent = {{-150, 150}, {150, 110}}, textString = "%name"), Text(origin = {10, -250}, extent = {{-150, 150}, {150, 110}}, textString = "%n"), Line(origin = {-36, -0.5}, points = {{0, -39.5}, {0, 8.5}, {0, 26.5}}, color = {170, 0, 255}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(origin = {-58, 43}, extent = {{-8, 13}, {8, -13}}, textString = "n")}));
end ProjectOnPlane;
