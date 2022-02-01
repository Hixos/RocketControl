within RocketControl.Simulations;

model Lynx
  extends Internal.Icon;
    parameter SI.Mass m = 28;
    parameter SI.Distance s_max = 0.0001;
    parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n * m / s_max;
    parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n * m / s_max;
    parameter SI.ModulusOfElasticity c_z = 2500 / s_max;
    parameter SI.ModulusOfElasticity d_x = 2 * sqrt(c_x * m);
    parameter SI.ModulusOfElasticity d_y = 2 * sqrt(c_y * m) * 4;
    parameter SI.ModulusOfElasticity d_z = 2 * sqrt(c_z * m) * 4;
    
  Rockets.Lynx.LynxRocket lynxRocket annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.Atmosphere atmosphere annotation(
      Placement(visible = true, transformation(origin = {-90, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.MyWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 2.268928027592628, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 1.466076571675237, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.ContinuousGNC continuousGNC annotation(
      Placement(visible = true, transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.GNC.Sensors.LynxSampledIdealSensors lynxSampledIdealSensors annotation(
      Placement(visible = true, transformation(origin = {50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.GNC.Sensors.LynxRealSensors lynxRealSensors annotation(
      Placement(visible = true, transformation(origin = {40, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.GNC.Navigation.LynxNavigation lynxNavigation annotation(
      Placement(visible = true, transformation(origin = {82, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(fixed.frame_b, launchRail.frame_a) annotation(
      Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_bow, lynxRocket.frame_lug_bow) annotation(
      Line(points = {{-40, 16}, {-20, 16}}, color = {95, 95, 95}));
    connect(launchRail.frame_b_lug_aft, lynxRocket.frame_lug_aft) annotation(
      Line(points = {{-40, 4}, {-20, 4}}));
    connect(lynxRocket.ref_center, landDetector.frame_a) annotation(
      Line(points = {{0, 10}, {54, 10}, {54, 90}, {80, 90}}, color = {95, 95, 95}));
    connect(lynxRocket.ref_center, continuousGNC.frame_a) annotation(
      Line(points = {{0, 10}, {20, 10}, {20, -30}, {40, -30}}, color = {95, 95, 95}));
    connect(lynxRocket.ref_center, lynxSampledIdealSensors.frame_a) annotation(
      Line(points = {{0, 10}, {20, 10}, {20, -70}, {40, -70}}));
    connect(lynxRealSensors.frame_a, lynxRocket.ref_center) annotation(
      Line(points = {{30, 46}, {20, 46}, {20, 10}, {0, 10}}));
  connect(lynxRealSensors.bus, lynxNavigation.bus) annotation(
      Line(points = {{50, 46}, {92, 46}, {92, 6}}, thickness = 0.5));
    annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-6, Interval = 0.01));
  end Lynx;
