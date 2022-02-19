within RocketControl.Components.Propulsion;

model M2000R
  extends RocketControl.Icons.RocketMotor;
  
  parameter Modelica.Units.SI.Time start_delay = 0;
  final parameter SI.Time Isp = 176;
  Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.1, 1500; 0.2, 2250; 2.1, 2400; 4, 1750; 4.6, 100; 4.7, 0; 500, 0], timeScale = 1) annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  RocketControl.Components.Propulsion.Internal.SolidMotor m2000r(Dext = 0.098, Din_0 = 0.03, Isp = Isp, h = 0.732, m_0 = 5.368) annotation(
    Placement(visible = true, transformation(origin = {22, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain mass_flow_rate(k = 1 / (Modelica.Constants.g_n * Isp)) annotation(
    Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(thrustCurve.y, mass_flow_rate.u) annotation(
    Line(points = {{-58, 0}, {-32, 0}}, color = {0, 0, 127}));
  connect(mass_flow_rate.y, m2000r.m_dot) annotation(
    Line(points = {{-8, 0}, {12, 0}}, color = {0, 0, 127}));
  connect(m2000r.frame_b, frame_b) annotation(
    Line(points = {{22, 10}, {20, 10}, {20, 40}, {0, 40}, {0, 98}}, color = {95, 95, 95}));
  annotation(
    Icon(graphics = {Text(origin = {0, -210}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "M2000R"), Line(origin = {0, 85}, points = {{0, 9}, {0, -9}})}));
end M2000R;
