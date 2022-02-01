within RocketControl.Simulations;

 model LynxLQ
  extends RocketControl.Icons.SimulationIcon;
    parameter SI.Mass m = 28;
    parameter SI.Distance s_max = 0.0001;
    parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n * m / s_max;
    parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n * m / s_max;
    parameter SI.ModulusOfElasticity c_z = 2500 / s_max;
    parameter SI.ModulusOfElasticity d_x = 2 * sqrt(c_x * m);
    parameter SI.ModulusOfElasticity d_y = 2 * sqrt(c_y * m) * 4;
    parameter SI.ModulusOfElasticity d_z = 2 * sqrt(c_z * m) * 4;
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.Atmosphere atmosphere annotation(
      Placement(visible = true, transformation(origin = {-90, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.FlatWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.466076571675237, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.ContinuousGNC continuousGNC annotation(
      Placement(visible = true, transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LynxWithCanardsRocket lynxWithCanardsRocket annotation(
      Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.GNC.Guidance.ConstantFlightPathGuidance constantFlightPathGuidance annotation(
    Placement(visible = true, transformation(origin = {10, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression control_enable(y = time > 1 and time < 8)  annotation(
    Placement(visible = true, transformation(origin = {-54, -34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.GNC.Control.BodyVelocityControl bodyVelocityControl annotation(
    Placement(visible = true, transformation(origin = {62, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lynxWithCanardsRocket.frame_lug_bow) annotation(
    Line(points = {{-40, 16}, {-10, 16}}));
  connect(launchRail.frame_b_lug_aft, lynxWithCanardsRocket.frame_lug_aft) annotation(
    Line(points = {{-40, 4}, {-10, 4}}, color = {95, 95, 95}));
  connect(lynxWithCanardsRocket.ref_center, continuousGNC.frame_a) annotation(
    Line(points = {{10, 10}, {31, 10}, {31, -30}, {40, -30}}, color = {95, 95, 95}));
  connect(lynxWithCanardsRocket.bus, continuousGNC.bus) annotation(
    Line(points = {{10, 17}, {84, 17}, {84, -30}, {60, -30}}, thickness = 0.5));
  connect(lynxWithCanardsRocket.ref_center, landDetector.frame_a) annotation(
    Line(points = {{10, 10}, {34, 10}, {34, 90}, {80, 90}}));
  connect(continuousGNC.bus, constantFlightPathGuidance.bus) annotation(
    Line(points = {{60, -30}, {78, -30}, {78, -64}, {-16, -64}, {-16, -84}, {0, -84}}, thickness = 0.5));
  connect(control_enable.y, lynxWithCanardsRocket.bus.control_enable) annotation(
    Line(points = {{-42, -34}, {10, -34}, {10, 18}}, color = {255, 0, 255}));
 connect(constantFlightPathGuidance.acc_err_int, bodyVelocityControl.vel_error) annotation(
    Line(points = {{22, -84}, {50, -84}}, color = {0, 0, 127}, thickness = 0.5));
 connect(bodyVelocityControl.bus, continuousGNC.bus) annotation(
    Line(points = {{72, -84}, {84, -84}, {84, -30}, {60, -30}}, thickness = 0.5));
protected
  annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-6, Interval = 0.01));
  end LynxLQ;
