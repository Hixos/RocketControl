within RocketControl.Simulations;

model LynxLQBench
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
  // ATMOSPHERE
  inner RocketControl.World.Atmosphere atmosphere(num_wind_layers = 1, wind_direction = from_deg({180}), wind_layer_height = {1000000}, wind_magnitude = {0}, wind_noise_sample_period = 0.1001, wind_severity_high = 1, wind_turbolence_enable = true, wing_span = 0.30)   annotation(
      Placement(visible = true, transformation(origin = {-90, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.FlatWorld world(altitude_0 = 1414, animateGravity = false, animateGround = false, animateWorld = true, enableAnimation = true, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = opt.launch_azimuth, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = opt.launch_elevation, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LynxOpenLoop rocket annotation(
    Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.SimOptions opt(drogue_enable = false, guidance_disable = false, guidance_disable_met = 10, guidance_enable_met = 1, launch_azimuth(displayUnit = "deg") = 2.268928027592628, launch_elevation = from_deg(84), main_enable = false, roll_target_heading = from_deg(90)) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.ContinuousGNC true_navigation annotation(
    Placement(visible = true, transformation(origin = {70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Blocks.Noise.GlobalSeed globalSeed(enableNoise = true, fixedSeed = 432532, useAutomaticSeed = false)  annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {64, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {58, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, rocket.frame_lug_bow) annotation(
    Line(points = {{-40, 16}, {-20, 16}}));
  connect(launchRail.frame_b_lug_aft, rocket.frame_lug_aft) annotation(
    Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
  connect(landDetector.frame_a, rocket.ref_center) annotation(
    Line(points = {{80, 90}, {0, 90}, {0, 10}}));
  connect(rocket.ref_center, true_navigation.frame_a) annotation(
    Line(points = {{0, 10}, {20, 10}, {20, 30}, {60, 30}}));
  connect(atmosphere.frame_a, rocket.ref_center) annotation(
    Line(points = {{-100, 88}, {-100, 40}, {0, 40}, {0, 10}}));
  connect(rocket.ref_center, realSensors.frame_a) annotation(
    Line(points = {{0, 10}, {28, 10}, {28, -30}, {54, -30}}, color = {95, 95, 95}));
  connect(realSensors.bus, navigation.bus) annotation(
    Line(points = {{74, -30}, {90, -30}, {90, -70}, {68, -70}}, color = {255, 204, 51}, thickness = 0.5));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 300, Tolerance = 1e-04, Interval = 0.01),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian -d=dumpLoops ",
  __OpenModelica_simulationFlags(s = "dassl"));
end LynxLQBench;
