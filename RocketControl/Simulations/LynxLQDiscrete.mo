within RocketControl.Simulations;

 model LynxLQDiscrete
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
  inner RocketControl.World.Atmosphere atmosphere(wind_speed = {20, 20, 0})  annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.FlatWorld world(altitude_0 = 100, animateGravity = false, animateGround = false, animateWorld = true, enableAnimation = true, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression control_enable(y = time > 1 and time < 15)  annotation(
    Placement(visible = true, transformation(origin = {-54, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = opt.launch_azimuth, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = opt.launch_elevation, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.LynxLinearAeroDiscrete lynxLinearAeroDiscrete annotation(
    Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.GNC.Guidance.ConstantFlightPathGuidanceDiscrete constantFlightPathGuidance(dt = opt.samplePeriodMs / 1000, flightpathangle = from_deg(83), heading = 0.5235987755982988, int_lim = 50, kint = 4)  annotation(
    Placement(visible = true, transformation(origin = {10, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Clocked.BooleanSignals.Sampler.Sample sample1 annotation(
    Placement(visible = true, transformation(origin = {-12, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
 RocketControl.GNC.Control.BodyVelocityControlDiscrete bodyVelocityControlDiscrete(Qvec = {0, 20, 20, 100, 0, 0, 0, 0, 0} * 0.01, Rvec = {1, 1, 0.4} * 300)  annotation(
    Placement(visible = true, transformation(origin = {70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 inner RocketControl.World.SimOptions opt(launch_azimuth = from_deg(180), launch_elevation = from_deg(60))  annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Components.Visualizers.AssetVisualizer assetVisualizer annotation(
    Placement(visible = true, transformation(origin = {14, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.GNC.Sensors.RealSensors realSensors annotation(
    Placement(visible = true, transformation(origin = {42, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lynxLinearAeroDiscrete.frame_lug_bow) annotation(
    Line(points = {{-40, 16}, {-10, 16}}));
  connect(launchRail.frame_b_lug_aft, lynxLinearAeroDiscrete.frame_lug_aft) annotation(
    Line(points = {{-40, 4}, {-10, 4}}, color = {95, 95, 95}));
  connect(landDetector.frame_a, lynxLinearAeroDiscrete.ref_center) annotation(
    Line(points = {{80, 90}, {30, 90}, {30, 10}, {10, 10}}));
  connect(control_enable.y, sample1.u) annotation(
    Line(points = {{-42, 42}, {-20, 42}, {-20, 60}}, color = {255, 0, 255}));
  connect(sample1.y, lynxLinearAeroDiscrete.bus.control_enable) annotation(
    Line(points = {{-6, 60}, {10, 60}, {10, 18}}, color = {255, 0, 255}));
  connect(constantFlightPathGuidance.acc_err_int, bodyVelocityControlDiscrete.vel_error) annotation(
    Line(points = {{21, -90}, {58, -90}}, color = {0, 0, 127}, thickness = 0.5));
  connect(navigation.bus, constantFlightPathGuidance.bus) annotation(
    Line(points = {{-40, -50}, {-40, -90}, {0, -90}}, thickness = 0.5));
  connect(assetVisualizer.frame_a, lynxLinearAeroDiscrete.ref_center) annotation(
    Line(points = {{4, 88}, {-2, 88}, {-2, 24}, {16, 24}, {16, 10}, {10, 10}}, color = {95, 95, 95}));
  connect(lynxLinearAeroDiscrete.ref_center, realSensors.frame_a) annotation(
    Line(points = {{10, 10}, {22, 10}, {22, -32}, {32, -32}}, color = {95, 95, 95}));
  connect(lynxLinearAeroDiscrete.bus, realSensors.bus) annotation(
    Line(points = {{10, 18}, {84, 18}, {84, -34}, {52, -34}, {52, -32}}, thickness = 0.5));
  connect(realSensors.bus, bodyVelocityControlDiscrete.bus) annotation(
    Line(points = {{52, -32}, {88, -32}, {88, -90}, {80, -90}}, thickness = 0.5));
  connect(realSensors.bus, navigation.bus) annotation(
    Line(points = {{52, -32}, {-40, -32}, {-40, -50}}, thickness = 0.5));
protected
  annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-6, Interval = 0.01));
  end LynxLQDiscrete;
