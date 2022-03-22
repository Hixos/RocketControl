within RocketControl.Simulations;

model LynxAeroCompare
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
  inner RocketControl.World.Atmosphere atmosphere(num_wind_layers = 2, wind_direction = from_deg({180, 0}), wind_layer_height = {150000, 100000}, wind_magnitude = {10, 10})   annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.FlatWorld world(altitude_0 = 1420, animateGravity = false, animateGround = false, animateWorld = true, enableAnimation = true, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = opt.launch_azimuth, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = opt.launch_elevation, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.LynxSimpleAeroSimplePara rocket annotation(
    Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.SimOptions opt(drogue_enable = false, guidance_disable = true, launch_azimuth(displayUnit = "deg") = 0, launch_elevation = from_deg(84)) annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.ContinuousGNC true_navigation annotation(
    Placement(visible = true, transformation(origin = {68, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression main_alt_expr(y = false) annotation(
    Placement(visible = true, transformation(origin = {-30, -59}, extent = {{-34, -7}, {34, 7}}, rotation = 0)));
  Modelica.Blocks.Sources.BooleanExpression apogee_expr(y = false) annotation(
    Placement(visible = true, transformation(origin = {-15, -45}, extent = {{-19, -7}, {19, 7}}, rotation = 0)));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = 20)  annotation(
    Placement(visible = true, transformation(origin = {-34, -140}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sample1(n = 4)  annotation(
    Placement(visible = true, transformation(origin = {-8, -102}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {-54, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
    Line(points = {{0, 10}, {58, 10}}));
  connect(apogee_expr.y, rocket.bus.drogue_deploy) annotation(
    Line(points = {{6, -44}, {14, -44}, {14, 20}, {0, 20}}, color = {255, 0, 255}));
  connect(main_alt_expr.y, rocket.bus.main_deploy) annotation(
    Line(points = {{7, -59}, {20, -59}, {20, 20}, {0, 20}}, color = {255, 0, 255}));
  connect(vectorConstant.v, sample1.u) annotation(
    Line(points = {{-42, -100}, {-28.5, -100}, {-28.5, -102}, {-15, -102}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sample1.y, rocket.bus.fin_setpoint) annotation(
    Line(points = {{-1, -102}, {36, -102}, {36, 20}, {0, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(periodicClock1.y, sample1.clock) annotation(
    Line(points = {{-28, -140}, {-8, -140}, {-8, -109}}, color = {175, 175, 175}));
  connect(rocket.ref_center, atmosphere.frame_a) annotation(
    Line(points = {{0, 10}, {0, 58}, {-100, 58}, {-100, 90}}, color = {95, 95, 95}));
protected
  annotation(
    Icon(coordinateSystem(grid = {2, 0})),
    experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-06, Interval = 0.01));
end LynxAeroCompare;
