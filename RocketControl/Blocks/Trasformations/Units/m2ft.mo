within RocketControl.Blocks.Trasformations.Units;

model m2ft
  parameter Integer n(min = 1) annotation(Evaluate = true);
  
  Modelica.Blocks.Interfaces.RealInput m[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput ft[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
 ft = m/0.3048;
annotation(
    Icon(graphics = {Rectangle(lineColor = {191, 0, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}}), Polygon(lineColor = {191, 0, 0}, fillColor = {191, 0, 0}, fillPattern = FillPattern.Solid, points = {{90, 0}, {30, 20}, {30, -20}, {90, 0}}), Text(extent = {{-36, -52}, {86, -88}}, textString = "ft", horizontalAlignment = TextAlignment.Right), Text(extent = {{-90, 86}, {32, 50}}, textString = "m", horizontalAlignment = TextAlignment.Left), Line(points = {{-90, 0}, {30, 0}}, color = {191, 0, 0})}, coordinateSystem(grid = {2, 0})));
end m2ft;
