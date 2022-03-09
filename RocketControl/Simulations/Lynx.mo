within RocketControl.Simulations;

model Lynx
  extends RocketControl.Icons.SimulationIcon;
    parameter SI.Mass m = 28;
    parameter SI.Distance s_max = 0.0001;
    parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n * m / s_max;
    parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n * m / s_max;
    parameter SI.ModulusOfElasticity c_z = 2500 / s_max;
    parameter SI.ModulusOfElasticity d_x = 2 * sqrt(c_x * m);
    parameter SI.ModulusOfElasticity d_y = 2 * sqrt(c_y * m) * 4;
    parameter SI.ModulusOfElasticity d_z = 2 * sqrt(c_z * m) * 4;
    
  RocketControl.Rockets.Lynx.LynxSimpleAero lynxRocket annotation(
      Placement(visible = true, transformation(origin = {-10, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.FlatWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = opt.launch_azimuth, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = opt.launch_elevation, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
      Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.Atmosphere atmosphere()  annotation(
    Placement(visible = true, transformation(origin = {-90, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.ContinuousGNC continuousGNC annotation(
    Placement(visible = true, transformation(origin = {70, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {50, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Rockets.Lynx.StateMachines.FlightModeManager flightModeManager annotation(
    Placement(visible = true, transformation(origin = {-10, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.SimOptions opt annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Blocks.Math.Vectors.VectorConstant vectorConstant(k = {0, 0, 0, 0}, n = 4)  annotation(
    Placement(visible = true, transformation(origin = {-56, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Clocked.RealSignals.Sampler.SampleVectorizedAndClocked sample1(n = 4)  annotation(
    Placement(visible = true, transformation(origin = {-24, 54}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicExactClock periodicClock1(factor = opt.samplePeriodMs)  annotation(
    Placement(visible = true, transformation(origin = {-54, 36}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
equation
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lynxRocket.frame_lug_bow) annotation(
    Line(points = {{-40, 16}, {-20, 16}}));
  connect(launchRail.frame_b_lug_aft, lynxRocket.frame_lug_aft) annotation(
    Line(points = {{-40, 4}, {-20, 4}}, color = {95, 95, 95}));
  connect(lynxRocket.ref_center, landDetector.frame_a) annotation(
    Line(points = {{0, 10}, {30, 10}, {30, 90}, {80, 90}}, color = {95, 95, 95}));
  connect(lynxRocket.ref_center, continuousGNC.frame_a) annotation(
    Line(points = {{0, 10}, {30, 10}, {30, 50}, {60, 50}}));
  connect(lynxRocket.ref_center, realSensors.frame_a) annotation(
    Line(points = {{0, 10}, {30, 10}, {30, -30}, {40, -30}}, color = {95, 95, 95}));
  connect(launchRail.liftoff, flightModeManager.bus.liftoff) annotation(
    Line(points = {{-44, 0}, {-44, -30}, {-20, -30}}, color = {255, 0, 255}));
  connect(lynxRocket.bus, realSensors.bus) annotation(
    Line(points = {{0, 20}, {30, 20}, {30, -10}, {80, -10}, {80, -30}, {60, -30}}, color = {255, 204, 51}, thickness = 0.5));
  connect(lynxRocket.bus, flightModeManager.bus) annotation(
    Line(points = {{0, 20}, {30, 20}, {30, -10}, {-32, -10}, {-32, -30}, {-20, -30}}, color = {255, 204, 51}, thickness = 0.5));
  connect(realSensors.bus, navigation.bus) annotation(
    Line(points = {{60, -30}, {80, -30}, {80, -60}, {60, -60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(vectorConstant.v, sample1.u) annotation(
    Line(points = {{-45, 60}, {-38, 60}, {-38, 54}, {-32, 54}}, color = {0, 0, 127}, thickness = 0.5));
  connect(sample1.y, lynxRocket.bus.fin_setpoint) annotation(
    Line(points = {{-18, 54}, {0, 54}, {0, 20}}, color = {0, 0, 127}, thickness = 0.5));
  connect(periodicClock1.y, sample1.clock) annotation(
    Line(points = {{-47, 36}, {-24, 36}, {-24, 46}}, color = {175, 175, 175}));
  annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-6, Interval = 0.01));
  end Lynx;
