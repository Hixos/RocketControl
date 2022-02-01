within RocketControl.Blocks.Flight;

block EulerRates
  extends Icon;
  Modelica.Blocks.Interfaces.RealInput w[3] annotation(
    Placement(visible = true, transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput q[4] annotation(
    Placement(visible = true, transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput euler_rates[3](each displayUnit = "deg/s", each final quantity = "AngularVelocity", each final unit = "rad/s") annotation(
    Placement(visible = true, transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  Real M[3, 3];
  Real a[3];
equation
  a = RocketControl.Math.quat2euler(q);
  if noEvent(abs(abs(a[2]) - Modelica.Constants.pi / 2) < 1e-3) then
    M = [0, sin(a[3]) / cos(1e-3 * sign(a[2])), cos(a[3]) / cos(1e-3 * sign(a[2])); 0, cos(a[3]), -sin(a[3]); 1, sin(a[3]) * tan(1e-3 * sign(a[2])), cos(a[3]) * tan(1e-3 * sign(a[2]))];
  else
    M = [0, sin(a[3]) / cos(a[2]), cos(a[3]) / cos(a[2]); 0, cos(a[3]), -sin(a[3]); 1, sin(a[3]) * tan(a[2]), cos(a[3]) * tan(a[2])];
  end if;
  euler_rates = M * w;
  annotation(
    Icon(graphics = {Line(origin = {9, 15.34}, points = {{-46, 29}, {-46, -45}, {20, -75}}, arrow = {Arrow.Filled, Arrow.Filled}, arrowSize = 10), Line(origin = {-2.8, 6.6}, points = {{-34, -35}, {36, 17}}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 10), Ellipse(origin = {-36, -28}, fillColor = {0, 85, 255}, fillPattern = FillPattern.Solid, extent = {{-12, 12}, {12, -12}}), Line(origin = {33.22, 23.49}, points = {{-29.1708, 16.1708}, {-17.1708, 20.1708}, {-3.17082, 20.1708}, {8.82918, 16.1708}, {16.8292, 8.1708}, {22.8292, -3.82918}, {22.8292, -27.8292}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Line(origin = {22.56, -59.08}, rotation = -90, points = {{-29.1708, 16.1708}, {-17.1708, 20.1708}, {-3.17082, 20.1708}, {8.82918, 16.1708}, {16.8292, 8.1708}, {22.8292, -3.82918}, {22.8292, -27.8292}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Line(origin = {-35.14, 35.84}, rotation = -90, points = {{-29.1708, 16.1708}, {-17.1708, 20.1708}, {-3.17082, 20.1708}, {8.82918, 16.1708}, {16.8292, 8.1708}, {22.8292, -3.82918}, {22.8292, -27.8292}}, color = {255, 0, 0}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 14), Text(origin = {-74, 61}, lineColor = {102, 102, 102}, extent = {{-26, 19}, {26, -19}}, textString = "w"), Text(origin = {-74, -59}, lineColor = {102, 102, 102}, extent = {{-26, 19}, {26, -19}}, textString = "q")}));
end EulerRates;
