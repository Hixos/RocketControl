within RocketControl;

model RocketSimRound
  import Modelica.Units.Conversions.from_deg;
  parameter SI.Mass m = 28;
  parameter SI.Distance s_max = 0.0001;
  parameter SI.ModulusOfElasticity c_x = Modelica.Constants.g_n * m / s_max;
  parameter SI.ModulusOfElasticity c_y = Modelica.Constants.g_n * m / s_max;
  parameter SI.ModulusOfElasticity c_z = 2500 / s_max;
  parameter SI.ModulusOfElasticity d_x = 2 * sqrt(c_x * m);
  parameter SI.ModulusOfElasticity d_y = 2 * sqrt(c_y * m) * 4;
  parameter SI.ModulusOfElasticity d_z = 2 * sqrt(c_z * m) * 4;
  inner RocketControl.World.Atmosphere atmosphere annotation(
    Placement(visible = true, transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner World.MyWorld world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.PointGravity, n = {0, 0, 1}) annotation(
    Placement(visible = true, transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Frames.ECEF ecef annotation(
    Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  RocketControl.Components.Frames.EarthSurfaceTranslation earthSurfaceTranslation(altitude = 300, latitude = 45, longitude = 8)  annotation(
    Placement(visible = true, transformation(origin = {-36, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body rocket(I_11 = 0.066590312, I_21 = -0.003190759, I_22 = 9.822815884, I_31 = 0.00128563, I_32 = -0.000234088, I_33 = 9.822815884, enforceStates = true, m = 22, r_CM = {0, 0, 0}, sequence_angleStates = {3, 2, 1}, w_a(start = {0, 0, 0})) annotation(
    Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_bow(r = {-0.02, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {70, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lug_aft(r = {-0.43, 0, -0.075}) annotation(
    Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Components.LaunchPad.LaunchRail launchRail(azimuth(displayUnit = "deg") = 0, c_x = c_x, c_y = c_y, c_z = c_z, d_x = d_x, d_y = d_y, d_z = d_z, elevation(displayUnit = "deg") = 85, lug_length = 0.04, r_rel = {0, 0, 0.04}, rail_length = 4) annotation(
    Placement(visible = true, transformation(origin = {28, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ecef.frame_b, earthSurfaceTranslation.frame_a) annotation(
    Line(points = {{-80, 10}, {-46, 10}}, color = {95, 95, 95}));
  connect(lug_bow.frame_a, rocket.frame_a) annotation(
    Line(points = {{80, 40}, {110, 40}, {110, 60}}, color = {95, 95, 95}));
  connect(lug_aft.frame_a, rocket.frame_a) annotation(
    Line(points = {{80, 0}, {80, 40}, {110, 40}, {110, 60}}, color = {95, 95, 95}));
  connect(earthSurfaceTranslation.frame_b, launchRail.frame_a) annotation(
    Line(points = {{-26, 10}, {18, 10}}, color = {95, 95, 95}));
  connect(launchRail.frame_b_lug_aft, lug_aft.frame_b) annotation(
    Line(points = {{38, 4}, {60, 4}, {60, 0}}));
  connect(launchRail.frame_b_lug_bow, lug_bow.frame_b) annotation(
    Line(points = {{38, 16}, {48, 16}, {48, 40}, {60, 40}}));
  annotation(
    experiment(StartTime = 0, StopTime = 100, Tolerance = 0.001, Interval = 0.01));
end RocketSimRound;
