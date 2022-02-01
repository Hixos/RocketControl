within RocketControl.Components.Propulsion;

model M2000R
  parameter Modelica.Units.SI.Time start_delay = 0;
  Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.1, 1500; 0.2, 2250; 2.1, 2400; 4, 1750; 4.6, 100; 4.7, 0; 500, 0], timeScale = 1) annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Internal.SolidMotorAndTank m2000r(Dext = 0.098, Din_0 = 0.03, Isp = 176, h = 0.732, m_0 = 5.368) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
equation
  connect(m2000r.frame_b, frame_b) annotation(
    Line(points = {{0, 10}, {0, 98}}));
  connect(thrustCurve.y, m2000r.thrust) annotation(
    Line(points = {{-59, 0}, {-10, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(origin = {2, -184}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Rectangle(origin = {0.51, 47.73}, fillColor = {85, 85, 85}, fillPattern = FillPattern.VerticalCylinder, extent = {{-59.94, 48.27}, {59.94, -48.27}}), Polygon(origin = {1, -30}, fillColor = {43, 88, 85}, fillPattern = FillPattern.VerticalCylinder, points = {{-21, 30}, {-51, 10}, {-71, -30}, {71, -30}, {49, 10}, {19, 30}, {5, 30}, {-21, 30}}), Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R")}));
end M2000R;
