within RocketControl.Simulations;

model LynxParachute
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
  inner World.Atmosphere atmosphere(wind_speed = {10, 10, 0}) annotation(
    Placement(visible = true, transformation(origin = {-90, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.FlatWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = opt.launch_azimuth, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = opt.launch_elevation, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.ContinuousGNC continuousGNC annotation(
    Placement(visible = true, transformation(origin = {50, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.SampledTrueSensors sampledTrueSensors annotation(
    Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {78, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {50, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.SimOptions opt(launch_azimuth = from_deg(0), launch_elevation = from_deg(84)) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.LynxParachute lynxRocket annotation(
    Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.StateMachines.FlightModeManager flightModeManager annotation(
    Placement(visible = true, transformation(origin = {-18, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(navigation.bus, realSensors.bus) annotation(
    Line(points = {{88, 22}, {100, 22}, {100, -90}, {60, -90}}, thickness = 0.5));
  connect(lynxRocket.bus, continuousGNC.bus) annotation(
    Line(points = {{0, 18}, {66, 18}, {66, -10}, {60, -10}}, thickness = 0.5));
  connect(realSensors.frame_a, lynxRocket.ref_center) annotation(
    Line(points = {{40, -90}, {20, -90}, {20, 10}, {0, 10}}, color = {95, 95, 95}));
  connect(lynxRocket.ref_center, sampledTrueSensors.frame_a) annotation(
    Line(points = {{0, 10}, {20, 10}, {20, -50}, {40, -50}}, color = {95, 95, 95}));
  connect(lynxRocket.ref_center, continuousGNC.frame_a) annotation(
    Line(points = {{0, 10}, {20, 10}, {20, -10}, {40, -10}}, color = {95, 95, 95}));
  connect(lynxRocket.ref_center, landDetector.frame_a) annotation(
    Line(points = {{0, 10}, {54, 10}, {54, 90}, {80, 90}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_aft, lynxRocket.frame_lug_aft) annotation(
    Line(points = {{-40, 4}, {-20, 4}}));
  connect(launchRail.frame_b_lug_bow, lynxRocket.frame_lug_bow) annotation(
    Line(points = {{-40, 16}, {-20, 16}}, color = {95, 95, 95}));
  connect(launchRail.liftoff, flightModeManager.bus.liftoff) annotation(
    Line(points = {{-44, 0}, {-44, -46}, {-28, -46}}, color = {255, 0, 255}));
  connect(lynxRocket.bus, flightModeManager.bus) annotation(
    Line(points = {{0, 18}, {10, 18}, {10, -30}, {-34, -30}, {-34, -46}, {-28, -46}}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 200, Tolerance = 1e-06, Interval = 0.01));
end LynxParachute;
