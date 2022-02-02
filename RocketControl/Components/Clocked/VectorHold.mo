within RocketControl.Components.Clocked;

model VectorHold

parameter Integer n(min = 1) annotation(Evaluate = true);
parameter Real y_start[n] = zeros(n);
  Modelica.Blocks.Interfaces.RealInput u[n] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y[n](start = y_start) annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
for i in 1:n loop
 y[i] = hold(u[i]);
 end for;
annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 130}, {150, 90}}, textString = "%name"), Line(points = {{-60, -40}, {-20, -40}, {-20, 20}, {20, 20}, {20, 60}, {60, 60}, {60, 0}, {100, 0}, {100, 0}, {100, 0}, {100, 0}, {120, 0}}, color = {0, 0, 127}), Line(points = {{-60, -40}, {-60, 0}, {-100, 0}}, color = {0, 0, 127}), Text(extent = {{-150, -100}, {150, -140}}, textString = "%y_start"), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-70, -30}, {-50, -50}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{-30, 28}, {-10, 8}}), Ellipse(lineColor = {0, 0, 127}, fillColor = {0, 0, 127}, fillPattern = FillPattern.Solid, extent = {{10, 70}, {30, 50}})}));
end VectorHold;
