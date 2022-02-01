within RocketControl.Components.Propulsion;

model L1350
  extends RocketControl.Icons.RocketMotor;
  parameter Modelica.Units.SI.Time start_delay = 0;
  Modelica.Blocks.Sources.TimeTable thrustCurve(offset = 0, shiftTime = start_delay, startTime = start_delay, table = [0, 0; 0.016, 1421.724; 0.034, 1345.218; 0.049, 1502.479; 0.081, 1415.348; 0.21, 1432.349; 0.453, 1432.349; 0.809, 1462.102; 1.07, 1534.357; 1.28, 1540.732; 2.661, 1283.589; 2.843, 1277.214; 2.932, 1115.702; 3.037, 488.784; 3.163, 82.881; 3.284, 0.0; 500, 0], timeScale = 1) annotation(
    Placement(visible = true, transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Internal.SolidMotorAndTank l1350cs(Dext = 0.075, Din_0 = 0.02, Isp = 228.2, h = 0.532, m_0 = 1.905) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
    Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
equation
  connect(l1350cs.frame_b, frame_b) annotation(
    Line(points = {{0, 10}, {0, 98}}));
  connect(thrustCurve.y, l1350cs.thrust) annotation(
    Line(points = {{-59, 0}, {-10, 0}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(origin = {2, -184}, lineColor = {0, 0, 255}, extent = {{-150, 110}, {150, 70}}, textString = "%name"), Text(origin = {1, 38}, lineColor = {255, 255, 255}, extent = {{-59, 38}, {59, -38}}, textString = "L1350")}));
end L1350;
