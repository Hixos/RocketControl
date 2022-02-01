within RocketControl.Blocks.Flight;

model ClimbAngle
  extends Icon;
  parameter SI.Velocity v_small = 1e-5;
  Modelica.Blocks.Interfaces.RealInput v[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput angle(displayUnit = "deg", quantity = "Angle", unit = "rad") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.Velocity vnorm;
equation
  vnorm = norm(v);
  if noEvent(vnorm > v_small) then
    angle = asin(v[3] / vnorm);
  else
    angle = 0;
  end if;
  annotation(
    Icon(graphics = {Ellipse(origin = {-60, 68}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {-15.86, -16.41}, points = {{76, 85}, {-44, 85}, {-44, -45}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Line(origin = {15.36, 4.44}, points = {{45, 64}, {45, -66}, {-75, -66}}, pattern = LinePattern.Dash), Line(origin = {91.73, -9.79}, points = {{-152, 79}, {-32, -51}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {10.1974, 19.1998}, points = {{0.8292, 48.1708}, {2.8292, 40.1708}, {2.82918, 30.1708}, {0.82918, 20.1708}, {-3.1708, 10.1708}, {-11.1708, 0.17082}, {-17.1708, -5.82918}}, color = {255, 0, 0}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Text(origin = {120, -26}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "glideSlope"), Text(origin = {-130, -32}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "v")}));
end ClimbAngle;
