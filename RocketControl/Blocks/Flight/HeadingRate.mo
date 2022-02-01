within RocketControl.Blocks.Flight;

block HeadingRate
  extends Icon;
  Modelica.Blocks.Interfaces.RealInput w[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q[4] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput heading_rate(final unit = "rad/s", final quantity = "AngularVelocity", displayUnit = "deg/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  SI.AngularVelocity w_yz[3];
  SI.AngularVelocity w_w[3];
equation
  w_yz = cat(1, {0}, w[2:3]);
  w_w = Modelica.Mechanics.MultiBody.Frames.Quaternions.resolve1(q, w_yz);
  heading_rate = w_w[3];
  annotation(
    Icon(graphics = {Line(origin = {11.7976, 8.19235}, points = {{-25.1708, 30.1708}, {-15.1708, 30.1708}, {-3.17082, 26.1708}, {4.82918, 22.1708}, {12.8292, 16.1708}, {22.8292, 6.17082}, {32.8292, -17.8292}}, color = {255, 0, 0}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Ellipse(origin = {-40, -60}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {-15.86, -16.41}, points = {{-16, -33}, {76, 85}}, color = {170, 0, 255}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Text(origin = {-79, 60}, lineColor = {120, 120, 120}, extent = {{-21, 20}, {21, -20}}, textString = "w"), Text(origin = {-79, -60}, lineColor = {120, 120, 120}, extent = {{-21, 20}, {21, -20}}, textString = "q")}));
end HeadingRate;
