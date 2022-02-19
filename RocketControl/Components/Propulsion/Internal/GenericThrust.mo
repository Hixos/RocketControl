within RocketControl.Components.Propulsion.Internal;

model GenericThrust
  parameter SI.Time Isp = 180 "Specific impulse in seconds";
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(
    Placement(visible = true, transformation(origin = {-34, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput m_dot annotation(
    Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-94, -2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain(k = Modelica.Constants.g_n * Isp) annotation(
    Placement(visible = true, transformation(origin = {-42, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(
    Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 94}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Blocks.Interfaces.RealOutput thrust annotation(
    Placement(visible = true, transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(force.frame_b, frame_b) annotation(
    Line(points = {{0, 60}, {0, 100}}, color = {95, 95, 95}));
  connect(m_dot, gain.u) annotation(
    Line(points = {{-100, 0}, {-54, 0}}, color = {0, 0, 127}));
  connect(gain.y, force.force[1]) annotation(
    Line(points = {{-30, 0}, {0, 0}, {0, 38}}, color = {0, 0, 127}));
  connect(gain.y, thrust) annotation(
    Line(points = {{-30, 0}, {106, 0}}, color = {0, 0, 127}));
  connect(const.y, force.force[2]) annotation(
    Line(points = {{-22, -54}, {0, -54}, {0, 38}}, color = {0, 0, 127}));
  connect(const.y, force.force[3]) annotation(
    Line(points = {{-22, -54}, {0, -54}, {0, 38}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Polygon(origin = {0, 12}, fillColor = {107, 107, 107}, fillPattern = FillPattern.VerticalCylinder, points = {{-6, 78}, {-60, 62}, {-90, 2}, {-100, -78}, {100, -78}, {90, 2}, {60, 62}, {6, 78}, {-6, 78}}), Text(origin = {-10, -172}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name")}));
end GenericThrust;
