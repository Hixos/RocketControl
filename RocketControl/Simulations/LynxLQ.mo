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
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(animation = false, r = {0, 0, -1.2}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.Atmosphere atmosphere(num_wind_layers = 3, wind_direction = from_deg({-180, 45, 0}), wind_layer_height = {1000, 100, 10}, wind_magnitude = {2, 20, 5})   annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.FlatWorld world(altitude_0 = 100, animateGravity = false, animateGround = false, animateWorld = true, enableAnimation = true, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = opt.launch_azimuth, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = opt.launch_elevation, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.LynxSimpleAeroSimplePara lynxLinearAeroDiscrete annotation(
    Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 inner RocketControl.World.SimOptions opt(drogue_enable = true, guidance_disable = true,launch_azimuth = from_deg(20), launch_elevation = from_deg(84))  annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Components.Visualizers.AssetVisualizer assetVisualizer annotation(
    Placement(visible = true, transformation(origin = {14, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.GNC.ContinuousGNC continuousGNC annotation(
    Placement(visible = true, transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.GNC.RealGNC realGNC annotation(
    Placement(visible = true, transformation(origin = {70, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.StateMachines.FlightModeManager flightModeManager annotation(
    Placement(visible = true, transformation(origin = {16, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lynxLinearAeroDiscrete.frame_lug_bow) annotation(
    Line(points = {{-40, 16}, {-20, 16}}));
  connect(launchRail.frame_b_lug_aft, lynxLinearAeroDiscrete.frame_lug_aft) annotation(
    Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
  connect(landDetector.frame_a, lynxLinearAeroDiscrete.ref_center) annotation(
    Line(points = {{80, 90}, {0, 90}, {0, 10}}));
  connect(assetVisualizer.frame_a, lynxLinearAeroDiscrete.ref_center) annotation(
    Line(points = {{4, 88}, {0, 88}, {0, 10}}, color = {95, 95, 95}));
  connect(continuousGNC.frame_a, lynxLinearAeroDiscrete.ref_center) annotation(
    Line(points = {{80, 50}, {48, 50}, {48, 10}, {0, 10}}, color = {95, 95, 95}));
  connect(lynxLinearAeroDiscrete.bus, realGNC.bus) annotation(
    Line(points = {{0, 20}, {80, 20}, {80, -30}}, thickness = 0.5));
  connect(lynxLinearAeroDiscrete.ref_center, realGNC.frame_a) annotation(
    Line(points = {{0, 10}, {38, 10}, {38, -30}, {60, -30}}));
 connect(lynxLinearAeroDiscrete.bus, flightModeManager.bus) annotation(
    Line(points = {{0, 20}, {6, 20}, {6, -52}}, color = {255, 204, 51}, thickness = 0.5));
 connect(launchRail.liftoff, flightModeManager.bus.liftoff) annotation(
    Line(points = {{-44, 0}, {-42, 0}, {-42, -52}, {6, -52}}, color = {255, 0, 255}));
protected
  annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-6, Interval = 0.01));
  end LynxLQ;
