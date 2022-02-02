within RocketControl.Blocks.Flight;

model Track
  extends Icon;
  Modelica.Blocks.Interfaces.RealInput v[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput track(final quantity = "Angle", final unit = "rad", displayUnit = "deg") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Real track_signed = 0;
equation
  if noEvent(norm(v) > 1) then
  if noEvent(abs(v[1]) > 1e-3) then
    track = atan2(v[2], v[1]);
  else
    track = sign(v[2]) * pi / 2;
  end if;
  else
  track = 0;
  end if;
  annotation(
    Icon(graphics = {Line(origin = {-15.86, -16.41}, points = {{-44, 85}, {-46, -45}, {76, -45}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Line(origin = {13.72, 25.15}, points = {{-73, 44}, {47, 44}, {47, -86}}, pattern = LinePattern.Dash), Line(origin = {-15.86, -16.41}, points = {{-46, -45}, {76, 85}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Line(origin = {-34.104, 15.4601}, points = {{-25.1708, 30.1708}, {-15.1708, 30.1708}, {-3.17082, 26.1708}, {4.82918, 22.1708}, {12.8292, 16.1708}, {22.8292, 6.17082}, {34.8292, -9.82918}}, color = {255, 0, 0}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Ellipse(origin = {-60, -60}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Text(origin = {-130, -32}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "v"), Text(origin = {120, -26}, lineColor = {120, 120, 120}, extent = {{-50, 20}, {50, -20}}, textString = "track")}));
end Track;
