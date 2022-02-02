within RocketControl.Components.Clocked;

model VectorSample

parameter Integer n(min = 1) annotation(Evaluate = true);
parameter Real u_start[n] = zeros(n);
  Modelica.Blocks.Interfaces.RealInput u[n](start = u_start) annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y[n] annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
for i in 1:n loop
 y[i] = sample(u[i]);
 end for;
annotation(
    Icon(graphics = {Ellipse(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{45, -10}, {25, 10}}), Text(lineColor = {0, 0, 255}, extent = {{-150, 90}, {150, 50}}, textString = "%name"), Ellipse(lineColor = {0, 0, 127}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-25, -10}, {-45, 10}}), Line(points = {{-100, 0}, {-45, 0}}, color = {0, 0, 127}), Line(points = {{45, 0}, {100, 0}}, color = {0, 0, 127}), Line(points = {{-35, 0}, {30, 35}}, color = {0, 0, 127})}, coordinateSystem(grid = {2, 0})));
end VectorSample;
