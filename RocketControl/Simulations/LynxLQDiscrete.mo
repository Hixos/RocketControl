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
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(r = {0, 0, -1.2}) annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.Parts.LandDetector landDetector annotation(
      Placement(visible = true, transformation(origin = {90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RocketControl.World.Atmosphere atmosphere annotation(
      Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.FlatWorld world(altitude_0 = 100, latitude_0 = 45.691051, longitude_0 = 8.490499, n = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Blocks.Sources.BooleanExpression control_enable(y = time > 1 and time < 8)  annotation(
    Placement(visible = true, transformation(origin = {-54, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = opt.launch_azimuth, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = opt.launch_elevation, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {-50, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.LynxLinearAeroDiscrete lynxLinearAeroDiscrete annotation(
    Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Rockets.Lynx.GNC.Sensors.SampledTrueSensors sampledTrueSensors annotation(
    Placement(visible = true, transformation(origin = {46, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.Rockets.Lynx.GNC.Navigation.Navigation navigation annotation(
    Placement(visible = true, transformation(origin = {4, -46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 RocketControl.GNC.Guidance.ConstantFlightPathGuidanceDiscrete constantFlightPathGuidance(dt = opt.samplePeriodMs / 1000, kint = 1)  annotation(
    Placement(visible = true, transformation(origin = {8, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Clocked.BooleanSignals.Sampler.Sample sample1 annotation(
    Placement(visible = true, transformation(origin = {-12, 60}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
 GNC.Control.BodyVelocityControlDiscrete bodyVelocityControlDiscrete(Qvec = {0, 20, 20, 100, 0, 0, 1, 1, 1} * 1, Rvec = {1, 1, 0.4} * 300)  annotation(
    Placement(visible = true, transformation(origin = {64, -86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 inner RocketControl.World.SimOptions opt annotation(
    Placement(visible = true, transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Rockets.Lynx.GNC.ContinuousGNC continuousGNC annotation(
    Placement(visible = true, transformation(origin = {100, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixed.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-80, 10}, {-60, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_bow, lynxLinearAeroDiscrete.frame_lug_bow) annotation(
    Line(points = {{-40, 16}, {-10, 16}}));
  connect(launchRail.frame_b_lug_aft, lynxLinearAeroDiscrete.frame_lug_aft) annotation(
    Line(points = {{-40, 4}, {-10, 4}}, color = {95, 95, 95}));
  connect(landDetector.frame_a, lynxLinearAeroDiscrete.ref_center) annotation(
    Line(points = {{80, 90}, {30, 90}, {30, 10}, {10, 10}}));
  connect(lynxLinearAeroDiscrete.ref_center, sampledTrueSensors.frame_a) annotation(
    Line(points = {{10, 10}, {22, 10}, {22, -24}, {36, -24}}));
  connect(sampledTrueSensors.bus, lynxLinearAeroDiscrete.bus) annotation(
    Line(points = {{56, -24}, {76, -24}, {76, 18}, {10, 18}}, thickness = 0.5));
  connect(navigation.bus, sampledTrueSensors.bus) annotation(
    Line(points = {{14, -46}, {68, -46}, {68, -24}, {56, -24}}, thickness = 0.5));
  connect(sampledTrueSensors.bus, constantFlightPathGuidance.bus) annotation(
    Line(points = {{56, -24}, {102, -24}, {102, -102}, {-4, -102}, {-4, -86}, {-2, -86}}, thickness = 0.5));
  connect(control_enable.y, sample1.u) annotation(
    Line(points = {{-42, 42}, {-20, 42}, {-20, 60}}, color = {255, 0, 255}));
  connect(sample1.y, lynxLinearAeroDiscrete.bus.control_enable) annotation(
    Line(points = {{-6, 60}, {10, 60}, {10, 18}}, color = {255, 0, 255}));
  connect(sampledTrueSensors.bus, bodyVelocityControlDiscrete.bus) annotation(
    Line(points = {{56, -24}, {90, -24}, {90, -86}, {74, -86}}, thickness = 0.5));
  connect(constantFlightPathGuidance.acc_err_int, bodyVelocityControlDiscrete.vel_error) annotation(
    Line(points = {{19, -86}, {52, -86}}, color = {0, 0, 127}, thickness = 0.5));
 connect(continuousGNC.frame_a, lynxLinearAeroDiscrete.ref_center) annotation(
    Line(points = {{90, 52}, {30, 52}, {30, 10}, {10, 10}}));
protected
  annotation(
      Icon(coordinateSystem(grid = {2, 0})),
      experiment(StartTime = 0, StopTime = 60, Tolerance = 1e-6, Interval = 0.01));
  end LynxLQDiscrete;
