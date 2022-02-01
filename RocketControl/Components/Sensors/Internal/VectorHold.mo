within RocketControl.Components.Sensors.Internal;

model VectorHold
  parameter Integer n(min = 1) = 1 "Number of signals to hold" annotation(
    Evaluate = true);
  Modelica.Blocks.Interfaces.RealOutput y[n] annotation(
    Placement(visible = true, transformation(extent = {{100, -10}, {120, 10}}, rotation = 0), iconTransformation(extent = {{100, -10}, {120, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u[n] annotation(
    Placement(visible = true, transformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0), iconTransformation(extent = {{-140, -20}, {-100, 20}}, rotation = 0)));
equation
  for i in 1:n loop
    y[i] = noEvent(hold(u[i]));
  end for;
  annotation(
    Icon(graphics = {Text(lineColor = {0, 0, 255}, extent = {{-150, 130}, {150, 90}}, textString = "%name"), Line(points = {{-60, -40}, {-60, 0}, {-100, 0}}, color = {0, 0, 127}), Line(points = {{-60, -40}, {-20, -40}, {-20, 20}, {20, 20}, {20, 60}, {60, 60}, {60, 0}, {100, 0}, {100, 0}, {100, 0}, {100, 0}, {120, 0}}, color = {0, 0, 127}), Text(origin = {0, -71}, lineColor = {86, 86, 86}, extent = {{-60, 21}, {60, -21}}, textString = "n = %n")}));
end VectorHold;
